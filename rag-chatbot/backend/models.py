from pydantic import BaseModel
from typing import Optional, List
from datetime import datetime


class ChatMessage(BaseModel):
    """Incoming chat message from user"""
    message: str
    session_id: str
    selected_text: Optional[str] = None
    page_url: Optional[str] = None


class ChatResponse(BaseModel):
    """Response sent back to user"""
    response: str
    sources: List[dict] = []
    session_id: str


class Document(BaseModel):
    """Document to be indexed"""
    content: str
    metadata: dict
    doc_id: Optional[str] = None


class ConversationHistory(BaseModel):
    """Conversation history entry"""
    session_id: str
    user_message: str
    bot_response: str
    timestamp: datetime
    page_url: Optional[str] = None
    selected_text: Optional[str] = None
