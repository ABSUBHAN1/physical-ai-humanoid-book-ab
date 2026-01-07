"""
Chapter model for the Physical AI & Humanoid Robotics textbook platform.
"""
from sqlalchemy import Column, String, Text, Integer, DateTime, ForeignKey, JSON
from sqlalchemy.sql import func
from .base import Base
import uuid


class Chapter(Base):
    __tablename__ = "chapters"

    id = Column(String, primary_key=True, default=lambda: str(uuid.uuid4()))
    module_id = Column(String, ForeignKey("modules.id"), nullable=False)
    title = Column(String, nullable=False)
    content = Column(Text)  # Markdown format
    order = Column(Integer, nullable=False)  # Sequence in module
    learning_objectives = Column(JSON, default=lambda: list())  # Array of strings
    estimated_duration = Column(Integer)  # Minutes
    hands_on_lab = Column(Text)  # Markdown format
    code_examples = Column(JSON, default=lambda: dict())  # Structured code examples
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now(), server_default=func.now())