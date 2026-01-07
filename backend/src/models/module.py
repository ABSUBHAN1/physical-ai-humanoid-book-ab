"""
Module model for the Physical AI & Humanoid Robotics textbook platform.
"""
from sqlalchemy import Column, String, Text, Integer, DateTime, JSON
from sqlalchemy.sql import func
from .base import Base
import uuid


class Module(Base):
    __tablename__ = "modules"

    id = Column(String, primary_key=True, default=lambda: str(uuid.uuid4()))
    title = Column(String, nullable=False)
    description = Column(Text)
    order = Column(Integer, nullable=False)  # Sequence in curriculum
    learning_objectives = Column(JSON, default=lambda: list())  # Array of strings
    prerequisites = Column(JSON, default=lambda: list())  # Array of Module IDs
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now(), server_default=func.now())