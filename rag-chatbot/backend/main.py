from fastapi import FastAPI, Request, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
import os
from dotenv import load_dotenv

from models import ChatMessage, ChatResponse
from rag import retrieve_context, generate_response, create_collection
from database import init_db, save_conversation, get_history

load_dotenv()

app = FastAPI(
    title="Physical AI Robotics Book Assistant",
    description="RAG-powered chatbot for the Physical AI & Humanoid Robotics book",
    version="1.0.0"
)

# CORS configuration
allowed_origins = os.getenv("ALLOWED_ORIGINS", "").split(",")
app.add_middleware(
    CORSMiddleware,
    allow_origins=allowed_origins if allowed_origins[0] else ["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.on_event("startup")
async def startup_event():
    """Initialize database and vector store on startup"""
    print("Initializing database...")
    init_db()
    print("Creating Qdrant collection...")
    create_collection()
    print("Backend ready!")


@app.get("/")
async def root():
    """Health check endpoint"""
    return {
        "status": "healthy",
        "service": "Physical AI Robotics Book Assistant",
        "version": "1.0.0"
    }


@app.get("/health")
async def health_check():
    """Detailed health check"""
    return {
        "status": "healthy",
        "database": "connected",
        "vector_store": "connected"
    }


@app.post("/chat", response_model=ChatResponse)
async def chat(chat_message: ChatMessage):
    """
    Main chat endpoint that processes messages with RAG

    Supports:
    - Regular chat messages
    - Selected text context
    - Conversation history
    """
    try:
        # Build query with selected text if provided
        query = chat_message.message
        if chat_message.selected_text:
            query = f"{chat_message.message}\n\nContext: {chat_message.selected_text}"

        # Retrieve context from vector store
        context, sources = retrieve_context(query, top_k=5)

        # Get conversation history
        history = get_history(chat_message.session_id, limit=5)

        # Generate response
        response_text = generate_response(
            user_message=chat_message.message,
            context=context,
            history=history,
            selected_text=chat_message.selected_text
        )

        # Save to database
        save_conversation(
            session_id=chat_message.session_id,
            user_message=chat_message.message,
            bot_response=response_text,
            page_url=chat_message.page_url,
            selected_text=chat_message.selected_text
        )

        return ChatResponse(
            response=response_text,
            sources=sources,
            session_id=chat_message.session_id
        )

    except Exception as e:
        print(f"Error in chat endpoint: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/history/{session_id}")
async def get_conversation_history(session_id: str, limit: int = 10):
    """Get conversation history for a session"""
    try:
        history = get_history(session_id, limit=limit)
        return {"session_id": session_id, "history": history}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/clear-history/{session_id}")
async def clear_conversation_history(session_id: str):
    """Clear conversation history for a session"""
    try:
        from database import clear_history
        clear_history(session_id)
        return {"message": f"History cleared for session {session_id}"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


if __name__ == "__main__":
    import uvicorn
    port = int(os.getenv("PORT", 8000))
    uvicorn.run(app, host="0.0.0.0", port=port)
