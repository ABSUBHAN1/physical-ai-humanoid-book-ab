"""
Embedding generation service for the Physical AI & Humanoid Robotics textbook platform.
This service handles the creation and management of vector embeddings for RAG functionality.
"""
import asyncio
from typing import List, Dict, Any, Optional
from uuid import uuid4
import numpy as np
from qdrant_client import QdrantClient, models
from transformers import AutoTokenizer, AutoModel
import torch
import hashlib

from .config import settings
from ..models import Chapter, LabExercise


class EmbeddingService:
    def __init__(self):
        # Initialize Qdrant client
        self.client = QdrantClient(
            host=settings.QDRANT_HOST,
            port=settings.QDRANT_PORT,
            api_key=settings.QDRANT_API_KEY
        )
        
        # Initialize transformer model for embeddings
        self.tokenizer = AutoTokenizer.from_pretrained(settings.EMBEDDING_MODEL)
        self.model = AutoModel.from_pretrained(settings.EMBEDDING_MODEL)
        
        # Create collection if it doesn't exist
        self._ensure_collection_exists()
    
    def _ensure_collection_exists(self):
        """Ensure the Qdrant collection exists with the correct configuration."""
        try:
            collections = self.client.get_collections()
            collection_names = [collection.name for collection in collections.collections]
            
            if settings.QDRANT_COLLECTION_NAME not in collection_names:
                self.client.create_collection(
                    collection_name=settings.QDRANT_COLLECTION_NAME,
                    vectors_config=models.VectorParams(
                        size=settings.EMBEDDING_DIMENSION,
                        distance=models.Distance.COSINE
                    )
                )
        except Exception as e:
            print(f"Error ensuring collection exists: {e}")
    
    def _generate_embedding(self, text: str) -> List[float]:
        """Generate embedding for a given text."""
        inputs = self.tokenizer(text, return_tensors="pt", truncation=True, padding=True, max_length=512)
        
        with torch.no_grad():
            outputs = self.model(**inputs)
            # Use mean pooling to get the sentence embedding
            embedding = outputs.last_hidden_state.mean(dim=1).squeeze().numpy()
        
        # Normalize the embedding
        embedding = embedding / np.linalg.norm(embedding)
        return embedding.tolist()
    
    def _split_text_into_chunks(self, text: str, chunk_size: int = 512, overlap: int = 50) -> List[str]:
        """Split text into overlapping chunks for better context retention."""
        tokens = self.tokenizer.tokenize(text)
        chunks = []
        
        for i in range(0, len(tokens), chunk_size - overlap):
            chunk_tokens = tokens[i:i + chunk_size]
            chunk_text = self.tokenizer.convert_tokens_to_string(chunk_tokens)
            chunks.append(chunk_text)
        
        return chunks
    
    def _generate_document_id(self, content_id: str, chunk_index: int, language: str = "en") -> str:
        """Generate a unique document ID for a content chunk."""
        unique_str = f"{content_id}_{chunk_index}_{language}"
        return hashlib.md5(unique_str.encode()).hexdigest()
    
    def add_content_to_index(self, content_id: str, content_type: str, content: str, 
                           language: str = "en", metadata: Optional[Dict[str, Any]] = None) -> bool:
        """
        Add content to the vector index.
        
        Args:
            content_id: Unique identifier for the content
            content_type: Type of content (e.g., "chapter", "lab_exercise")
            content: The actual content text
            language: Language of the content
            metadata: Additional metadata to store with the content
        
        Returns:
            True if successful, False otherwise
        """
        try:
            # Split content into chunks
            chunks = self._split_text_into_chunks(content)
            
            points = []
            for idx, chunk in enumerate(chunks):
                # Generate embedding for the chunk
                embedding = self._generate_embedding(chunk)
                
                # Create a unique ID for this chunk
                point_id = self._generate_document_id(content_id, idx, language)
                
                # Prepare metadata
                chunk_metadata = {
                    "content_id": content_id,
                    "content_type": content_type,
                    "chunk_index": idx,
                    "language": language,
                    "chunk_text": chunk[:100] + "..." if len(chunk) > 100 else chunk  # Store preview
                }
                
                if metadata:
                    chunk_metadata.update(metadata)
                
                # Create a point for Qdrant
                point = models.PointStruct(
                    id=point_id,
                    vector=embedding,
                    payload=chunk_metadata
                )
                
                points.append(point)
            
            # Upload all points to Qdrant
            self.client.upsert(
                collection_name=settings.QDRANT_COLLECTION_NAME,
                points=points
            )
            
            return True
        except Exception as e:
            print(f"Error adding content to index: {e}")
            return False
    
    def search_similar_content(self, query: str, top_k: int = 5, language: str = "en") -> List[Dict[str, Any]]:
        """
        Search for similar content in the vector index.
        
        Args:
            query: Query text to search for
            top_k: Number of results to return
            language: Language to filter results by
        
        Returns:
            List of similar content chunks with metadata
        """
        try:
            query_embedding = self._generate_embedding(query)
            
            results = self.client.search(
                collection_name=settings.QDRANT_COLLECTION_NAME,
                query_vector=query_embedding,
                query_filter=models.Filter(
                    must=[
                        models.FieldCondition(
                            key="language",
                            match=models.MatchValue(value=language)
                        )
                    ]
                ),
                limit=top_k,
                with_payload=True
            )
            
            # Format results
            formatted_results = []
            for result in results:
                formatted_results.append({
                    "id": result.id,
                    "score": result.score,
                    "content": result.payload.get("chunk_text", ""),
                    "metadata": {
                        "content_id": result.payload.get("content_id"),
                        "content_type": result.payload.get("content_type"),
                        "chunk_index": result.payload.get("chunk_index"),
                        "language": result.payload.get("language")
                    }
                })
            
            return formatted_results
        except Exception as e:
            print(f"Error searching for similar content: {e}")
            return []
    
    def delete_content_from_index(self, content_id: str) -> bool:
        """
        Delete content from the vector index.
        
        Args:
            content_id: ID of the content to delete
        
        Returns:
            True if successful, False otherwise
        """
        try:
            # Find all points with this content_id
            search_result = self.client.scroll(
                collection_name=settings.QDRANT_COLLECTION_NAME,
                scroll_filter=models.Filter(
                    must=[
                        models.FieldCondition(
                            key="content_id",
                            match=models.MatchValue(value=content_id)
                        )
                    ]
                ),
                limit=10000  # Assuming no content has more than 10k chunks
            )
            
            # Extract point IDs
            point_ids = [point.id for point in search_result[0]]
            
            if point_ids:
                # Delete the points
                self.client.delete(
                    collection_name=settings.QDRANT_COLLECTION_NAME,
                    points_selector=models.PointIdsList(points=point_ids)
                )
            
            return True
        except Exception as e:
            print(f"Error deleting content from index: {e}")
            return False
    
    def process_new_content(self, db_session, content_type: str):
        """
        Process all new content of a given type and add it to the vector index.
        
        Args:
            db_session: Database session
            content_type: Type of content to process ("chapter" or "lab_exercise")
        """
        if content_type == "chapter":
            # Get all chapters
            chapters = db_session.query(Chapter).all()
            
            for chapter in chapters:
                # Add main content
                self.add_content_to_index(
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
                    self.add_content_to_index(
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
        
        elif content_type == "lab_exercise":
            # Get all lab exercises
            lab_exercises = db_session.query(LabExercise).all()
            
            for lab in lab_exercises:
                # Add lab exercise content
                self.add_content_to_index(
                    content_id=f"lab_{lab.id}",
                    content_type="lab_exercise",
                    content=f"{lab.description}\n\nCode Template:\n{lab.code_template}\n\nSolution:\n{lab.solution}",
                    language="en",
                    metadata={
                        "title": lab.title,
                        "chapter_id": lab.chapter_id
                    }
                )


# Global embedding service instance
embedding_service = EmbeddingService()