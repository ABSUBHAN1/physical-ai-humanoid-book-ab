"""
UserProgress model for the Physical AI & Humanoid Robotics textbook platform.
"""
from sqlalchemy import Column, String, Integer, DateTime, ForeignKey
from sqlalchemy.sql import func
from .base import Base
import uuid


class UserProgress(Base):
    __tablename__ = "user_progress"

    id = Column(String, primary_key=True, default=lambda: str(uuid.uuid4()))
    user_id = Column(String, ForeignKey("users.id"), nullable=False)
    chapter_id = Column(String, ForeignKey("chapters.id"), nullable=False)
    status = Column(String, default="not-started")  # Options: "not-started", "in-progress", "completed"
    completion_percentage = Column(Integer, default=0)
    time_spent = Column(Integer)  # Seconds
    last_accessed = Column(DateTime(timezone=True), onupdate=func.now())
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now(), server_default=func.now())