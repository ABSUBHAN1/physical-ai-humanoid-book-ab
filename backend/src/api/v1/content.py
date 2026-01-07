"""
Content API router for the Physical AI & Humanoid Robotics textbook platform.
"""
from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.orm import Session
from typing import Optional

from ..middleware.auth import get_current_user
from ..models import User
from ..database import get_db
from ..services.content_service import ContentService

router = APIRouter()

@router.get("/modules")
async def get_all_modules(
    language: Optional[str] = "en",
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db)
):
    """
    Retrieve a list of all available modules.
    """
    content_service = ContentService(db)
    modules = content_service.get_all_modules(language=language)
    return {"modules": modules}


@router.get("/modules/{module_id}")
async def get_module_details(
    module_id: str,
    language: Optional[str] = "en",
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db)
):
    """
    Retrieve detailed information about a specific module.
    """
    content_service = ContentService(db)
    module_details = content_service.get_module_details(module_id=module_id, language=language)

    if not module_details:
        raise HTTPException(status_code=404, detail="Module not found")

    return module_details


@router.get("/chapters/{chapter_id}")
async def get_chapter_content(
    chapter_id: str,
    language: Optional[str] = "en",
    personalize: Optional[bool] = True,
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db)
):
    """
    Retrieve content for a specific chapter.
    """
    content_service = ContentService(db)
    chapter_content = content_service.get_chapter_content(
        chapter_id=chapter_id,
        language=language,
        personalize=personalize
    )

    if not chapter_content:
        raise HTTPException(status_code=404, detail="Chapter not found")

    return chapter_content


@router.get("/translate/{chapter_id}")
async def translate_content(
    chapter_id: str,
    target_language: str,
    source_language: Optional[str] = "en",
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db)
):
    """
    Get translated content for a chapter.
    """
    # Implementation will be added in later phases
    return {
        "chapter_id": chapter_id,
        "source_language": source_language,
        "target_language": target_language,
        "title": "کیا فزیکل اے آئی ہے؟",
        "content": "# کیا فزیکل اے آئی ہے؟\n\nفزیکل اے آئی ایک نظریہ ہے...",
        "hands_on_lab": {
            "title": "بنیادی فزیکل اے آئی سیمیولیشن",
            "description": "ایک سادہ فزیکل اے آئی سیمیولیشن لاگو کریں"
        }
    }


@router.get("/search")
async def search_content(
    q: str,
    language: Optional[str] = "en",
    limit: Optional[int] = 10,
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db)
):
    """
    Search across all content.
    """
    content_service = ContentService(db)
    search_results = content_service.search_content(
        query=q,
        language=language,
        limit=limit
    )

    return {
        "query": q,
        "results": search_results
    }