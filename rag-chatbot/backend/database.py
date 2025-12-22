import os
from datetime import datetime
from typing import List, Optional
from sqlalchemy import create_engine, Column, String, DateTime, Text
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
from dotenv import load_dotenv

load_dotenv()

# Database setup
DATABASE_URL = os.getenv("NEON_DATABASE_URL")
engine = create_engine(DATABASE_URL)
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)
Base = declarative_base()


class Conversation(Base):
    """Conversation history table"""
    __tablename__ = "conversations"

    id = Column(String, primary_key=True)
    session_id = Column(String, index=True, nullable=False)
    user_message = Column(Text, nullable=False)
    bot_response = Column(Text, nullable=False)
    timestamp = Column(DateTime, default=datetime.utcnow)
    page_url = Column(String, nullable=True)
    selected_text = Column(Text, nullable=True)


def init_db():
    """Initialize database tables"""
    Base.metadata.create_all(bind=engine)


def save_conversation(
    session_id: str,
    user_message: str,
    bot_response: str,
    page_url: Optional[str] = None,
    selected_text: Optional[str] = None
) -> None:
    """Save a conversation to the database"""
    db = SessionLocal()
    try:
        conversation = Conversation(
            id=f"{session_id}_{datetime.utcnow().timestamp()}",
            session_id=session_id,
            user_message=user_message,
            bot_response=bot_response,
            page_url=page_url,
            selected_text=selected_text
        )
        db.add(conversation)
        db.commit()
    finally:
        db.close()


def get_history(session_id: str, limit: int = 5) -> List[dict]:
    """Get conversation history for a session"""
    db = SessionLocal()
    try:
        conversations = (
            db.query(Conversation)
            .filter(Conversation.session_id == session_id)
            .order_by(Conversation.timestamp.desc())
            .limit(limit)
            .all()
        )

        return [
            {
                "user": conv.user_message,
                "assistant": conv.bot_response,
                "timestamp": conv.timestamp.isoformat(),
                "page_url": conv.page_url,
                "selected_text": conv.selected_text
            }
            for conv in reversed(conversations)
        ]
    finally:
        db.close()


def clear_history(session_id: str) -> None:
    """Clear conversation history for a session"""
    db = SessionLocal()
    try:
        db.query(Conversation).filter(
            Conversation.session_id == session_id
        ).delete()
        db.commit()
    finally:
        db.close()
