"""
Main API application entry point for the Physical AI & Humanoid Robotics textbook platform.
"""
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from contextlib import asynccontextmanager

from src.api.v1.chatbot import router as chatbot_router
from src.api.v1.content import router as content_router
from src.api.v1.user import router as user_router
from src.api.v1.translation import router as translation_router
from src.database import init_db


@asynccontextmanager
async def lifespan(app: FastAPI):
    """
    Handle startup and shutdown events.
    """
    # Startup
    print("Starting up Physical AI & Humanoid Robotics textbook platform...")
    # Initialize the database
    init_db()
    print("Database initialized successfully.")

    yield

    # Shutdown
    print("Shutting down Physical AI & Humanoid Robotics textbook platform...")


def create_app() -> FastAPI:
    """Create and configure the FastAPI application."""
    app = FastAPI(
        title="Physical AI & Humanoid Robotics Textbook API",
        description="API for the Physical AI & Humanoid Robotics textbook platform with RAG, personalization, and translation features.",
        version="1.0.0",
        lifespan=lifespan
    )

    # Add CORS middleware
    app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"],  # In production, restrict this to your frontend domain
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )

    # Include API routers
    app.include_router(chatbot_router, prefix="/api/v1/chatbot", tags=["chatbot"])
    app.include_router(content_router, prefix="/api/v1/content", tags=["content"])
    app.include_router(user_router, prefix="/api/v1/user", tags=["user"])
    app.include_router(translation_router, prefix="/api/v1/translation", tags=["translation"])

    @app.get("/")
    def read_root():
        return {"message": "Welcome to the Physical AI & Humanoid Robotics Textbook API"}

    return app


app = create_app()


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)