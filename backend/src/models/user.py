"""
User model for the Physical AI & Humanoid Robotics textbook platform.
"""
from sqlalchemy import Column, String, DateTime, JSON
from sqlalchemy.sql import func
from .base import Base
import uuid


class User(Base):
    __tablename__ = "users"

    id = Column(String, primary_key=True, default=lambda: str(uuid.uuid4()))
    email = Column(String, unique=True, nullable=False)
    name = Column(String, nullable=False)
    hashed_password = Column(String, nullable=False)
    learning_preferences = Column(JSON, default=lambda: dict())
    language_preference = Column(String, default="en")  # Options: "en", "ur"
    technical_background = Column(String, default="beginner")  # Options: "beginner", "intermediate", "advanced"
    progress = Column(JSON, default=lambda: dict())
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now(), server_default=func.now())