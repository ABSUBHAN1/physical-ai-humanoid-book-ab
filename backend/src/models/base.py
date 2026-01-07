"""
Database models for the Physical AI & Humanoid Robotics textbook platform.
"""
from sqlalchemy import Column, Integer, String, Text, DateTime, ForeignKey, JSON
from sqlalchemy.orm import declarative_base, relationship
from sqlalchemy.sql import func
from sqlalchemy import create_engine
from sqlalchemy.ext.hybrid import hybrid_property
import uuid

Base = declarative_base()


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

    # Relationships
    user_progress = relationship("UserProgress", back_populates="user")
    chat_sessions = relationship("ChatSession", back_populates="user")


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

    # Relationships
    chapters = relationship("Chapter", back_populates="module")


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

    # Relationships
    module = relationship("Module", back_populates="chapters")
    lab_exercises = relationship("LabExercise", back_populates="chapter")
    user_progress = relationship("UserProgress", back_populates="chapter")
    translations = relationship("ChapterTranslation", back_populates="chapter")


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

    # Relationships
    chapter = relationship("Chapter", back_populates="lab_exercises")


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

    # Relationships
    user = relationship("User", back_populates="user_progress")
    chapter = relationship("Chapter", back_populates="user_progress")


class ChatSession(Base):
    __tablename__ = "chat_sessions"

    id = Column(String, primary_key=True, default=lambda: str(uuid.uuid4()))
    user_id = Column(String, ForeignKey("users.id"), nullable=False)
    module_id = Column(String, ForeignKey("modules.id"))  # Optional
    chapter_id = Column(String, ForeignKey("chapters.id"))  # Optional
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now(), server_default=func.now())

    # Relationships
    user = relationship("User", back_populates="chat_sessions")
    messages = relationship("ChatMessage", back_populates="session")


class ChatMessage(Base):
    __tablename__ = "chat_messages"

    id = Column(String, primary_key=True, default=lambda: str(uuid.uuid4()))
    session_id = Column(String, ForeignKey("chat_sessions.id"), nullable=False)
    role = Column(String, nullable=False)  # Options: "user", "assistant"
    content = Column(Text, nullable=False)
    timestamp = Column(DateTime(timezone=True), server_default=func.now())
    context_chunks = Column(JSON, default=lambda: list())  # Array of strings (relevant content chunks)

    # Relationships
    session = relationship("ChatSession", back_populates="messages")


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

    # Relationships
    chapter = relationship("Chapter", back_populates="translations")