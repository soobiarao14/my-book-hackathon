# Quick Start Guide - RAG Chatbot

## Current Status

✅ **Completed:**
- Python dependencies installation in progress
- Backend API created (FastAPI + RAG)
- Chat widget integrated into Docusaurus
- Cohere, Qdrant, and Neon Postgres configured

⚠️ **Required Before Running:**
- Add OpenAI API key to `.env` file

## Step 1: Add OpenAI API Key

Edit `rag-chatbot/.env` and add your OpenAI API key:

```env
OPENAI_API_KEY=sk-proj-your-actual-key-here
```

Get your API key at: https://platform.openai.com/api-keys

## Step 2: Start the Chatbot Backend

```bash
cd rag-chatbot
python backend/main.py
```

The backend will run on: **http://localhost:8000**

Test it: http://localhost:8000/health

##Step 3: Index Your Book Content

In a new terminal:

```bash
cd rag-chatbot
python ingest.py --build-dir ../book-site/build
```

This will:
- Scan all HTML files in your Docusaurus build
- Create vector embeddings with Cohere
- Index them into Qdrant Cloud

Expected output: "✅ Successfully indexed X document chunks!"

## Step 4: Start Docusaurus

In another terminal:

```bash
cd book-site
npm start
```

Your site will open at: **http://localhost:3000**

## Step 5: Test the Chatbot!

1. Look for the **purple chat button** in the bottom-right corner
2. Click it to open the chat widget
3. Try asking:
   - "What is ROS 2?"
   - "Explain Vision-Language-Action models"
   - Or highlight any text and ask "Explain this"

## Troubleshooting

### "Backend not responding"
- Make sure backend is running: `curl http://localhost:8000/health`
- Check `.env` has valid OpenAI API key

### "No results found"
- Run ingestion: `python ingest.py --build-dir ../book-site/build`
- Check Qdrant credentials in `.env`

### Widget not showing
- Check browser console for errors
- Verify `Root.tsx` exists in `book-site/src/theme/`
- Clear browser cache and restart dev server

## What's Next?

### Deploy to Production

1. **Backend**: Deploy to Render.com (see `rag-chatbot/README.md`)
2. **Frontend**: Already setup! Push to GitHub and it auto-deploys
3. **Update Backend URL**: Change in `ChatbotWidget.tsx` and `chatbot-widget.html`

### Customize

- **Colors**: Edit gradients in `ChatbotWidget.tsx`
- **Model**: Change in `backend/rag.py` (GPT-4 → GPT-3.5 for cheaper)
- **Chunk Size**: Adjust in `.env`

##Files Created

```
my-book/
├── rag-chatbot/                    ← RAG Backend
│   ├── backend/
│   │   ├── main.py                # FastAPI server
│   │   ├── rag.py                 # RAG logic
│   │   ├── database.py            # Postgres
│   │   └── requirements.txt
│   ├── ingest.py                  # Content indexer
│   ├── .env                       # Your credentials
│   └── README.md                  # Full documentation
│
├── book-site/
│   ├── src/
│   │   ├── components/
│   │   │   └── ChatbotWidget.tsx  ← React widget
│   │   └── theme/
│   │       └── Root.tsx           ← Theme wrapper
│   └── ...
│
└── QUICK_START.md                 ← This file
```

## Features

- ✅ Smart RAG-based answers (no hallucinations)
- ✅ Selected text support
- ✅ Conversation history
- ✅ Beautiful gradient UI
- ✅ Mobile responsive
- ✅ Source citations

---

**Need help?** Check `rag-chatbot/README.md` for detailed documentation!
