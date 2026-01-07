"""
RAG (Retrieval-Augmented Generation) service for the Physical AI & Humanoid Robotics textbook platform.
"""
import asyncio
from typing import List, Dict, Any, Optional
from sqlalchemy.orm import Session

from ..core.config import settings
from ..core.embeddings import embedding_service
from ..models import Chapter, User, ChatSession, ChatMessage
from ..database import get_db


class RAGService:
    def __init__(self):
        self.embedding_service = embedding_service
    
    async def get_relevant_content(self, query: str, top_k: int = 5, language: str = "en") -> List[Dict[str, Any]]:
        """
        Retrieve relevant content based on the query using vector similarity search.
        
        Args:
            query: The user's query
            top_k: Number of results to return
            language: Language of the content to retrieve
        
        Returns:
            List of relevant content chunks with metadata
        """
        return self.embedding_service.search_similar_content(
            query=query,
            top_k=top_k,
            language=language
        )
    
    async def generate_response(self, query: str, context_chunks: List[Dict[str, Any]], user_id: str) -> str:
        """
        Generate a response using the LLM with retrieved context.
        
        Args:
            query: The user's query
            context_chunks: Relevant content chunks retrieved from the vector store
            user_id: ID of the user asking the question
        
        Returns:
            Generated response string
        """
        # In a real implementation, this would call an LLM API like OpenAI
        # For now, we'll return a mock response that includes the context
        
        # Combine the context chunks into a single context string
        context = "\n\n".join([chunk["content"] for chunk in context_chunks])
        
        # Mock response generation - in reality, this would call an LLM
        response = f"Based on the textbook content, here's an answer to your question: '{query}'\n\n"
        response += f"Key points from the textbook:\n{context[:500]}..."  # Truncate for brevity
        
        return response
    
    async def get_response(self, query: str, user_id: str, session_id: str) -> str:
        """
        Main method to get a response to a user's query using RAG.
        
        Args:
            query: The user's query
            user_id: ID of the user asking the question
            session_id: ID of the chat session
        
        Returns:
            Generated response string
        """
        # Retrieve relevant content based on the query
        relevant_content = await self.get_relevant_content(
            query=query,
            top_k=settings.RAG_TOP_K,
            language="en"  # Would be based on user's language preference in a real implementation
        )
        
        # Generate response using the retrieved content
        response = await self.generate_response(query, relevant_content, user_id)
        
        # In a real implementation, we would also store the interaction in the database
        # and potentially update user preferences based on the interaction
        
        return response
    
    async def process_new_content(self, db_session: Session, content_type: str, content_id: str):
        """
        Process new content and add it to the vector index.
        
        Args:
            db_session: Database session
            content_type: Type of content ("chapter", "lab_exercise", etc.)
            content_id: ID of the specific content to process
        """
        if content_type == "chapter":
            # Get the specific chapter
            chapter = db_session.query(Chapter).filter(Chapter.id == content_id).first()
            if chapter:
                # Add main content
                self.embedding_service.add_content_to_index(
                    content_id=f"chapter_{chapter.id}",
                    content_type="chapter",
                    content=chapter.content or "",
                    language="en",
                    metadata={
                        "title": chapter.title,
                        "module_id": chapter.module_id
                    }
                )
                
                # Add hands-on lab content if available
                if chapter.hands_on_lab:
                    self.embedding_service.add_content_to_index(
                        content_id=f"chapter_{chapter.id}_lab",
                        content_type="chapter_lab",
                        content=chapter.hands_on_lab,
                        language="en",
                        metadata={
                            "title": f"{chapter.title} - Lab Exercise",
                            "module_id": chapter.module_id,
                            "parent_content_id": chapter.id
                        }
                    )