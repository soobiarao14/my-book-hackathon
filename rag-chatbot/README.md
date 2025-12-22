# RAG Chatbot for Physical AI & Humanoid Robotics Book

A production-ready Retrieval-Augmented Generation (RAG) chatbot that allows readers to ask questions about your Docusaurus book. Features selected-text querying, conversation history, and beautiful UI.

## Features

- **Smart RAG System**: Answers based only on your book content (no hallucinations)
- **Selected Text Support**: Highlight any text on the page and ask questions about it
- **Conversation History**: Full chat history stored in Neon Postgres
- **Beautiful UI**: Professional floating widget with dark/light mode support
- **Mobile Responsive**: Works seamlessly on all devices

## Tech Stack

- **Backend**: FastAPI (Python)
- **Embeddings**: Cohere API (`embed-english-v3.0`)
- **Vector DB**: Qdrant Cloud
- **Database**: Neon Serverless Postgres
- **LLM**: OpenAI GPT-4 Turbo
- **Frontend**: React/TypeScript (Docusaurus component)

## Project Structure

```
rag-chatbot/
├── backend/
│   ├── main.py              # FastAPI server
│   ├── rag.py               # RAG logic (Cohere + Qdrant)
│   ├── database.py          # Neon Postgres operations
│   ├── models.py            # Pydantic models
│   └── requirements.txt     # Python dependencies
├── chatkit-widget/
│   └── chatbot-widget.html  # Standalone HTML widget
├── ingest.py                # Content indexing script
├── .env                     # Environment variables (FILLED)
├── .env.example             # Template
└── README.md                # This file
```

## Quick Start

### 1. Prerequisites

- Python 3.9+
- Node.js 18+ (for Docusaurus)
- API Keys:
  - ✅ Cohere API key (already provided)
  - ✅ Qdrant Cloud URL & API key (already provided)
  - ✅ Neon Postgres connection string (already provided)
  - ⚠️ OpenAI API key (you need to add this)

### 2. Setup Backend

```bash
# Navigate to rag-chatbot directory
cd rag-chatbot

# Install Python dependencies
pip install -r backend/requirements.txt

# Update .env file with your OpenAI API key
# Open .env and add:
# OPENAI_API_KEY=sk-your-key-here
```

### 3. Index Your Content

This step crawls your Docusaurus build and creates vector embeddings.

```bash
# First, build your Docusaurus site
cd ../book-site
npm run build

# Then run the ingestion script
cd ../rag-chatbot
python ingest.py --build-dir ../book-site/build
```

**Expected Output:**
```
Scanning ../book-site/build for HTML files...
Found 42 HTML files
Processing: index.html
Processing: module-1-ros2.html
...
✅ Successfully indexed 312 document chunks!
```

### 4. Start the Backend

```bash
# Make sure you're in rag-chatbot directory
python backend/main.py
```

The backend will start on `http://localhost:8000`

**Test it:**
```bash
curl http://localhost:8000/health
# Should return: {"status":"healthy","database":"connected","vector_store":"connected"}
```

### 5. Integrate Widget into Docusaurus

**Option A: Using the React Component (Recommended)**

1. The component is already created at: `book-site/src/components/ChatbotWidget.tsx`

2. Add it to your Docusaurus theme by editing `book-site/src/theme/Root.tsx`:

```tsx
import React from 'react';
import ChatbotWidget from '@site/src/components/ChatbotWidget';

export default function Root({children}) {
  return (
    <>
      {children}
      <ChatbotWidget />
    </>
  );
}
```

3. If `Root.tsx` doesn't exist, create it:

```bash
mkdir -p book-site/src/theme
# Then create the file with the content above
```

**Option B: Using Standalone HTML**

Copy the content from `chatkit-widget/chatbot-widget.html` and paste it into your Docusaurus custom HTML.

### 6. Update Backend URL for Production

Before deploying, update the backend URL in:

- `book-site/src/components/ChatbotWidget.tsx` (line ~13)
- `chatkit-widget/chatbot-widget.html` (line ~226)

Change from:
```javascript
const BACKEND_URL = 'http://localhost:8000';
```

To your deployed backend URL:
```javascript
const BACKEND_URL = 'https://your-backend.onrender.com';
```

## Deployment

### Deploy Backend (Render)

1. Create account at [render.com](https://render.com)
2. Click "New +" → "Web Service"
3. Connect your GitHub repository
4. Configure:
   - **Name**: `robotics-book-chatbot`
   - **Runtime**: Python 3
   - **Build Command**: `cd rag-chatbot && pip install -r backend/requirements.txt`
   - **Start Command**: `cd rag-chatbot/backend && uvicorn main:app --host 0.0.0.0 --port $PORT`
5. Add Environment Variables:
   - `COHERE_API_KEY`
   - `QDRANT_URL`
   - `QDRANT_API_KEY`
   - `NEON_DATABASE_URL`
   - `OPENAI_API_KEY`
   - `ALLOWED_ORIGINS=https://soobiarao14.github.io`
6. Deploy!

### Deploy Frontend (Already Setup)

Your Docusaurus site with the chatbot widget will deploy automatically via GitHub Actions to:
**https://soobiarao14.github.io/my-book-hackathon/**

## Usage

### For Readers

1. Click the purple chat button in the bottom-right corner
2. Ask questions like:
   - "How do I install ROS 2?"
   - "Explain the difference between Gazebo and Isaac Sim"
   - "What are Vision-Language-Action models?"
3. **Pro Tip**: Highlight any text on the page, then ask "Explain this in simpler terms"

### Re-indexing Content

After updating your book:

```bash
cd book-site
npm run build

cd ../rag-chatbot
python ingest.py --build-dir ../book-site/build
```

### Clearing Conversation History

```bash
curl -X POST http://localhost:8000/clear-history/SESSION_ID
```

## Environment Variables Reference

| Variable | Description | Example |
|----------|-------------|---------|
| `COHERE_API_KEY` | Cohere embeddings API key | `pyrZHJ4LSh...` |
| `QDRANT_URL` | Qdrant Cloud endpoint | `https://xyz.qdrant.cloud:6333` |
| `QDRANT_API_KEY` | Qdrant authentication | `eyJhbGci...` |
| `NEON_DATABASE_URL` | Postgres connection string | `postgresql://user:pass@...` |
| `OPENAI_API_KEY` | OpenAI GPT-4 key | `sk-proj-...` |
| `BACKEND_URL` | Backend API endpoint | `http://localhost:8000` |
| `ALLOWED_ORIGINS` | CORS allowed domains | `https://soobiarao14.github.io` |

## API Endpoints

### `POST /chat`
Send a message and get AI response.

**Request:**
```json
{
  "message": "What is ROS 2?",
  "session_id": "session_123",
  "selected_text": "Optional highlighted text",
  "page_url": "https://yoursite.com/docs/ros2"
}
```

**Response:**
```json
{
  "response": "ROS 2 is a robotics middleware...",
  "sources": [
    {
      "content": "ROS 2 documentation excerpt...",
      "score": 0.89,
      "source": "docs/module-1-ros2.html",
      "title": "Module 1: ROS 2 Foundations"
    }
  ],
  "session_id": "session_123"
}
```

### `GET /history/{session_id}`
Get conversation history.

### `POST /clear-history/{session_id}`
Clear conversation history.

### `GET /health`
Backend health check.

## Troubleshooting

### "Backend not responding"
- Check if backend is running: `curl http://localhost:8000/health`
- Verify CORS settings in `.env` → `ALLOWED_ORIGINS`

### "No results found"
- Re-run ingestion: `python ingest.py --build-dir ../book-site/build`
- Check Qdrant collection exists

### "Database connection error"
- Verify Neon Postgres URL in `.env`
- Test connection: `psql $NEON_DATABASE_URL`

### Widget not showing
- Check browser console for errors
- Verify `ChatbotWidget.tsx` is imported in `Root.tsx`
- Clear browser cache

## Customization

### Change Widget Colors

Edit the gradient in `ChatbotWidget.tsx` or `chatbot-widget.html`:

```css
background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
```

### Adjust Response Length

In `backend/rag.py`, modify:

```python
max_tokens=800  # Change this value
```

### Change Model

In `backend/rag.py`:

```python
model="gpt-4-turbo-preview"  # or "gpt-3.5-turbo" for faster/cheaper
```

## Security Notes

- Never commit `.env` file (already in `.gitignore`)
- Rotate API keys regularly
- Use HTTPS for production deployments
- Implement rate limiting for production (see FastAPI docs)

## Cost Estimates (per 1000 queries)

- Cohere Embeddings: ~$0.10
- Qdrant Cloud: Free tier (100K vectors)
- Neon Postgres: Free tier (0.5GB)
- OpenAI GPT-4 Turbo: ~$5-10 (depending on response length)

**Monthly Cost for ~10K queries: $50-100**

## Support

For issues related to:
- **This integration**: Check console logs, verify environment variables
- **Cohere API**: https://docs.cohere.com
- **Qdrant**: https://qdrant.tech/documentation
- **Neon**: https://neon.tech/docs
- **OpenAI**: https://platform.openai.com/docs

## License

MIT License - feel free to use this in your own projects!

## Credits

Built with:
- [FastAPI](https://fastapi.tiangolo.com/)
- [Cohere](https://cohere.com/)
- [Qdrant](https://qdrant.tech/)
- [Neon](https://neon.tech/)
- [OpenAI](https://openai.com/)

---

**Your RAG chatbot is ready! 🚀**

Next steps:
1. Add your OpenAI API key to `.env`
2. Run `python ingest.py --build-dir ../book-site/build`
3. Start backend: `python backend/main.py`
4. Add widget to Docusaurus (see step 5 above)
5. Deploy to production!
