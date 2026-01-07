"""
ChapterTranslation model for the Physical AI & Humanoid Robotics textbook platform.
"""
from sqlalchemy import Column, String, Text, DateTime, ForeignKey
from sqlalchemy.sql import func
from .base import Base
import uuid


class ChapterTranslation(Base):
    __tablename__ = "chapter_translations"

    id = Column(String, primary_key=True, default=lambda: str(uuid.uuid4()))
    chapter_id = Column(String, ForeignKey("chapters.id"), nullable=False)
    language = Column(String, nullable=False)  # e.g., "ur", "en"
    title = Column(String)
    content = Column(Text)  # Markdown format
    hands_on_lab = Column(Text)  # Markdown format
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now(), server_default=func.now())