"""
Content Ingestion Script for Physical AI Robotics Book

This script crawls the Docusaurus build directory and indexes all content
into Qdrant using Cohere embeddings.
"""

import os
import json
from pathlib import Path
from typing import List, Dict
from bs4 import BeautifulSoup
import re
from dotenv import load_dotenv
import sys

# Add backend to path
sys.path.append(str(Path(__file__).parent / "backend"))

from rag import index_documents, create_collection

load_dotenv()


def extract_text_from_html(html_content: str) -> str:
    """Extract clean text from HTML"""
    soup = BeautifulSoup(html_content, 'html.parser')

    # Remove script and style elements
    for script in soup(["script", "style", "nav", "footer"]):
        script.decompose()

    # Get text
    text = soup.get_text()

    # Clean up whitespace
    lines = (line.strip() for line in text.splitlines())
    chunks = (phrase.strip() for line in lines for phrase in line.split("  "))
    text = ' '.join(chunk for chunk in chunks if chunk)

    return text


def chunk_text(text: str, chunk_size: int = 512, overlap: int = 50) -> List[str]:
    """Split text into overlapping chunks"""
    words = text.split()
    chunks = []

    for i in range(0, len(words), chunk_size - overlap):
        chunk = ' '.join(words[i:i + chunk_size])
        if chunk:
            chunks.append(chunk)

    return chunks


def extract_metadata_from_path(file_path: Path, base_path: Path) -> Dict:
    """Extract metadata from file path"""
    relative_path = file_path.relative_to(base_path)

    # Try to extract title from path
    parts = str(relative_path).replace('\\', '/').split('/')
    title = parts[-1].replace('.html', '').replace('-', ' ').title()

    # Determine module/section
    module = "General"
    if len(parts) > 1:
        if "module-1" in str(relative_path):
            module = "Module 1: ROS 2 Foundations"
        elif "module-2" in str(relative_path):
            module = "Module 2: Digital Twin & Simulation"
        elif "module-3" in str(relative_path):
            module = "Module 3: NVIDIA Isaac"
        elif "module-4" in str(relative_path):
            module = "Module 4: Vision-Language-Action"
        elif "appendices" in str(relative_path):
            module = "Appendices"

    return {
        "source": str(relative_path),
        "title": title,
        "module": module,
        "url_path": "/" + str(relative_path).replace('\\', '/')
    }


def ingest_docusaurus_site(build_dir: str):
    """Ingest all HTML files from Docusaurus build directory"""
    build_path = Path(build_dir)

    if not build_path.exists():
        print(f"Error: Build directory not found: {build_dir}")
        print("Please run 'npm run build' in the book-site directory first.")
        return

    print(f"Scanning {build_dir} for HTML files...")

    # Find all HTML files
    html_files = list(build_path.rglob("*.html"))
    print(f"Found {len(html_files)} HTML files")

    documents = []

    for html_file in html_files:
        try:
            # Skip certain files
            if any(skip in str(html_file) for skip in ['404.html', 'search.html']):
                continue

            print(f"Processing: {html_file.name}")

            # Read and extract text
            with open(html_file, 'r', encoding='utf-8') as f:
                html_content = f.read()

            text = extract_text_from_html(html_content)

            if len(text) < 100:  # Skip very short pages
                continue

            # Get metadata
            metadata = extract_metadata_from_path(html_file, build_path)

            # Chunk the text
            chunks = chunk_text(text, chunk_size=512, overlap=50)

            # Create documents for each chunk
            for i, chunk in enumerate(chunks):
                doc_metadata = metadata.copy()
                doc_metadata["chunk_index"] = i
                doc_metadata["total_chunks"] = len(chunks)

                documents.append({
                    "content": chunk,
                    "metadata": doc_metadata
                })

        except Exception as e:
            print(f"Error processing {html_file}: {e}")

    print(f"\nPrepared {len(documents)} document chunks from {len(html_files)} pages")

    # Create collection and index documents
    print("\nCreating Qdrant collection...")
    create_collection()

    print("Indexing documents into Qdrant...")
    # Index in batches to avoid timeout
    batch_size = 100
    for i in range(0, len(documents), batch_size):
        batch = documents[i:i + batch_size]
        print(f"Indexing batch {i//batch_size + 1}/{(len(documents)-1)//batch_size + 1}")
        index_documents(batch)

    print(f"\n✅ Successfully indexed {len(documents)} document chunks!")
    print(f"Your RAG chatbot is now ready to answer questions about the book.")


def ingest_markdown_files(docs_dir: str):
    """Ingest markdown files directly (alternative approach)"""
    docs_path = Path(docs_dir)

    if not docs_path.exists():
        print(f"Error: Docs directory not found: {docs_dir}")
        return

    print(f"Scanning {docs_dir} for Markdown files...")

    # Find all markdown files
    md_files = list(docs_path.rglob("*.md"))
    print(f"Found {len(md_files)} Markdown files")

    documents = []

    for md_file in md_files:
        try:
            print(f"Processing: {md_file.name}")

            # Read markdown content
            with open(md_file, 'r', encoding='utf-8') as f:
                content = f.read()

            # Extract title from frontmatter or filename
            title = md_file.stem.replace('-', ' ').title()

            # Simple metadata
            metadata = {
                "source": str(md_file.relative_to(docs_path)),
                "title": title,
                "type": "markdown"
            }

            # Chunk the content
            chunks = chunk_text(content, chunk_size=512, overlap=50)

            for i, chunk in enumerate(chunks):
                doc_metadata = metadata.copy()
                doc_metadata["chunk_index"] = i

                documents.append({
                    "content": chunk,
                    "metadata": doc_metadata
                })

        except Exception as e:
            print(f"Error processing {md_file}: {e}")

    print(f"\n✅ Prepared {len(documents)} chunks from {len(md_files)} files")

    # Index documents
    create_collection()
    index_documents(documents)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Ingest content for RAG chatbot")
    parser.add_argument(
        "--build-dir",
        default="../book-site/build",
        help="Path to Docusaurus build directory"
    )
    parser.add_argument(
        "--markdown",
        action="store_true",
        help="Ingest from markdown files instead of build"
    )
    parser.add_argument(
        "--docs-dir",
        default="../book-site/docs",
        help="Path to docs directory (if using --markdown)"
    )

    args = parser.parse_args()

    if args.markdown:
        ingest_markdown_files(args.docs_dir)
    else:
        ingest_docusaurus_site(args.build_dir)
