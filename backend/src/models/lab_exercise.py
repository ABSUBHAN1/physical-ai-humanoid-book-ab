"""
LabExercise model for the Physical AI & Humanoid Robotics textbook platform.
"""
from sqlalchemy import Column, String, Text, Integer, DateTime, ForeignKey
from sqlalchemy.sql import func
from .base import Base
import uuid


class LabExercise(Base):
    __tablename__ = "lab_exercises"

    id = Column(String, primary_key=True, default=lambda: str(uuid.uuid4()))
    chapter_id = Column(String, ForeignKey("chapters.id"), nullable=False)
    title = Column(String, nullable=False)
    description = Column(Text)
    simulation_environment = Column(String)  # Options: "gazebo", "unity", "nvidia-isaac", "none"
    code_template = Column(Text)
    solution = Column(Text)
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now(), server_default=func.now())