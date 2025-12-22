import os
import cohere
from openai import OpenAI
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct
from typing import List, Dict, Tuple
from dotenv import load_dotenv

load_dotenv()

# Initialize clients
cohere_client = cohere.Client(os.getenv("COHERE_API_KEY"))
openai_client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))
qdrant_client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY")
)

COLLECTION_NAME = os.getenv("COLLECTION_NAME", "robotics_book_docs")
EMBEDDING_MODEL = os.getenv("EMBEDDING_MODEL", "embed-english-v3.0")


def create_collection():
    """Create Qdrant collection if it doesn't exist"""
    try:
        # Try to get collection info (will fail if doesn't exist)
        collection = qdrant_client.collection_exists(COLLECTION_NAME)
        if collection:
            print(f"Collection '{COLLECTION_NAME}' already exists")
        else:
            qdrant_client.create_collection(
                collection_name=COLLECTION_NAME,
                vectors_config=VectorParams(size=1024, distance=Distance.COSINE)
            )
            print(f"Created collection '{COLLECTION_NAME}'")
    except Exception as e:
        # Collection already exists or other error
        print(f"Collection check/creation: {str(e)[:100]}")
        print(f"Assuming collection '{COLLECTION_NAME}' exists, continuing...")


def get_embeddings(texts: List[str]) -> List[List[float]]:
    """Get embeddings from Cohere"""
    response = cohere_client.embed(
        texts=texts,
        model=EMBEDDING_MODEL,
        input_type="search_document"
    )
    return response.embeddings


def get_query_embedding(query: str) -> List[float]:
    """Get embedding for a search query"""
    response = cohere_client.embed(
        texts=[query],
        model=EMBEDDING_MODEL,
        input_type="search_query"
    )
    return response.embeddings[0]


def index_documents(documents: List[Dict]) -> None:
    """Index documents into Qdrant"""
    texts = [doc["content"] for doc in documents]
    embeddings = get_embeddings(texts)

    points = [
        PointStruct(
            id=i,
            vector=embedding,
            payload={
                "content": doc["content"],
                "metadata": doc.get("metadata", {})
            }
        )
        for i, (doc, embedding) in enumerate(zip(documents, embeddings))
    ]

    qdrant_client.upsert(
        collection_name=COLLECTION_NAME,
        points=points
    )
    print(f"Indexed {len(documents)} documents")


def retrieve_context(query: str, top_k: int = 5) -> Tuple[str, List[dict]]:
    """Retrieve relevant context from Qdrant"""
    query_embedding = get_query_embedding(query)

    search_results = qdrant_client.search(
        collection_name=COLLECTION_NAME,
        query_vector=query_embedding,
        limit=top_k
    )

    # Build context string
    context_parts = []
    sources = []

    for result in search_results:
        content = result.payload["content"]
        metadata = result.payload.get("metadata", {})

        context_parts.append(content)
        sources.append({
            "content": content[:200] + "..." if len(content) > 200 else content,
            "score": result.score,
            "source": metadata.get("source", "Unknown"),
            "title": metadata.get("title", "Untitled")
        })

    context = "\n\n---\n\n".join(context_parts)
    return context, sources


def generate_response(
    user_message: str,
    context: str,
    history: List[dict],
    selected_text: str = None
) -> str:
    """Generate response using OpenAI with RAG context"""

    # Build system prompt
    system_prompt = f"""You are an expert AI assistant for the "Physical AI & Humanoid Robotics" book.
Your role is to help readers understand concepts related to ROS 2, Gazebo simulation, Vision-Language-Action models, and humanoid robotics.

Use the following context from the book to answer questions accurately:

{context}

Guidelines:
- Answer based ONLY on the provided context
- If the answer isn't in the context, say "I don't have enough information in the book to answer that"
- Be concise but thorough
- Use technical terms when appropriate, but explain them clearly
- Reference specific modules or chapters when relevant"""

    if selected_text:
        system_prompt += f"\n\nThe user has selected this text from the page:\n\"{selected_text}\"\nUse this as additional context for your response."

    # Build messages
    messages = [{"role": "system", "content": system_prompt}]

    # Add conversation history
    for hist in history[-5:]:  # Last 5 exchanges
        messages.append({"role": "user", "content": hist["user"]})
        messages.append({"role": "assistant", "content": hist["assistant"]})

    # Add current message
    messages.append({"role": "user", "content": user_message})

    # Generate response
    response = openai_client.chat.completions.create(
        model="gpt-4-turbo-preview",
        messages=messages,
        temperature=0.7,
        max_tokens=800
    )

    return response.choices[0].message.content
