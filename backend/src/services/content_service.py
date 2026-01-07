"""
Content service for the Physical AI & Humanoid Robotics textbook platform.
"""
from typing import List, Dict, Any, Optional
from sqlalchemy.orm import Session

from ..models import Module, Chapter, LabExercise, ChapterTranslation
from ..database import get_db


class ContentService:
    def __init__(self, db_session: Session):
        self.db = db_session
    
    def get_all_modules(self, language: str = "en") -> List[Dict[str, Any]]:
        """
        Retrieve all available modules.
        
        Args:
            language: Language of the content to retrieve
        
        Returns:
            List of module dictionaries
        """
        modules = self.db.query(Module).order_by(Module.order).all()
        
        result = []
        for module in modules:
            # Count chapters in this module
            chapter_count = self.db.query(Chapter).filter(Chapter.module_id == module.id).count()
            
            result.append({
                "id": module.id,
                "title": module.title,
                "description": module.description,
                "order": module.order,
                "learning_objectives": module.learning_objectives,
                "estimated_duration": chapter_count * 30,  # Assuming ~30 min per chapter
                "progress": {
                    "completed_chapters": 0,  # This would come from user progress in a real implementation
                    "total_chapters": chapter_count
                }
            })
        
        return result
    
    def get_module_details(self, module_id: str, language: str = "en") -> Optional[Dict[str, Any]]:
        """
        Retrieve detailed information about a specific module.
        
        Args:
            module_id: ID of the module
            language: Language of the content to retrieve
        
        Returns:
            Module details dictionary or None if not found
        """
        module = self.db.query(Module).filter(Module.id == module_id).first()
        if not module:
            return None
        
        chapters = self.db.query(Chapter).filter(Chapter.module_id == module_id).order_by(Chapter.order).all()
        
        chapter_list = []
        for chapter in chapters:
            chapter_list.append({
                "id": chapter.id,
                "title": chapter.title,
                "order": chapter.order,
                "estimated_duration": chapter.estimated_duration,
                "learning_objectives": chapter.learning_objectives,
                "progress": {
                    "status": "not-started",  # This would come from user progress in a real implementation
                    "completion_percentage": 0
                }
            })
        
        return {
            "id": module.id,
            "title": module.title,
            "description": module.description,
            "order": module.order,
            "learning_objectives": module.learning_objectives,
            "prerequisites": module.prerequisites,
            "chapters": chapter_list
        }
    
    def get_chapter_content(self, chapter_id: str, language: str = "en", personalize: bool = True) -> Optional[Dict[str, Any]]:
        """
        Retrieve content for a specific chapter.
        
        Args:
            chapter_id: ID of the chapter
            language: Language of the content to retrieve
            personalize: Whether to personalize content based on user preferences
        
        Returns:
            Chapter content dictionary or None if not found
        """
        chapter = self.db.query(Chapter).filter(Chapter.id == chapter_id).first()
        if not chapter:
            return None
        
        # Get translated content if available
        translated_content = None
        if language != "en":
            translation = self.db.query(ChapterTranslation).filter(
                ChapterTranslation.chapter_id == chapter_id,
                ChapterTranslation.language == language
            ).first()
            
            if translation:
                translated_content = {
                    "title": translation.title,
                    "content": translation.content,
                    "hands_on_lab": translation.hands_on_lab
                }
        
        # Use translated content if available, otherwise use original
        if translated_content:
            title = translated_content["title"] or chapter.title
            content = translated_content["content"] or chapter.content or ""
            hands_on_lab = translated_content["hands_on_lab"] or chapter.hands_on_lab or ""
        else:
            title = chapter.title
            content = chapter.content or ""
            hands_on_lab = chapter.hands_on_lab or ""
        
        # Get lab exercises for this chapter
        lab_exercises = self.db.query(LabExercise).filter(LabExercise.chapter_id == chapter_id).all()
        
        return {
            "id": chapter.id,
            "module_id": chapter.module_id,
            "title": title,
            "content": content,
            "learning_objectives": chapter.learning_objectives,
            "estimated_duration": chapter.estimated_duration,
            "hands_on_lab": {
                "title": f"{title} - Hands-on Lab",
                "description": hands_on_lab,
                "simulation_environment": "gazebo",  # This would come from the lab exercise record
                "code_template": "",  # This would come from the lab exercise record
                "solution": ""  # This would come from the lab exercise record
            },
            "code_examples": chapter.code_examples,
            "review_questions": [
                {
                    "question": "What are the key concepts from this chapter?",
                    "type": "short-answer",
                    "options": []
                }
            ],
            "glossary": [
                {
                    "term": "Example Term",
                    "definition": "Example definition of a term from this chapter"
                }
            ]
        }
    
    def search_content(self, query: str, language: str = "en", limit: int = 10) -> List[Dict[str, Any]]:
        """
        Search across all content.
        
        Args:
            query: Search query string
            language: Language of the content to search
            limit: Maximum number of results to return
        
        Returns:
            List of search results
        """
        # This is a basic implementation - in a real system, this would use the embedding service
        # to perform semantic search rather than simple text matching
        
        # For now, we'll just return some mock results
        results = []
        
        # In a real implementation, we would:
        # 1. Use the embedding service to find semantically similar content
        # 2. Or use full-text search on the database
        # 3. Combine and rank results
        
        # Mock implementation returning some example results
        if "physical ai" in query.lower():
            results.append({
                "id": "chapter_1",
                "type": "chapter",
                "title": "What is Physical AI?",
                "content_preview": "Physical AI represents a paradigm shift from traditional digital AI to embodied intelligence...",
                "relevance_score": 0.95,
                "url": "/docs/01_introduction/01_what_is_physical_ai"
            })
        
        if "embodied" in query.lower():
            results.append({
                "id": "chapter_2",
                "type": "chapter",
                "title": "Why Embodied Intelligence?",
                "content_preview": "Embodied intelligence is the foundation of Physical AI because it recognizes that intelligence emerges...",
                "relevance_score": 0.90,
                "url": "/docs/01_introduction/02_why_embodied_intelligence"
            })
        
        return results[:limit]