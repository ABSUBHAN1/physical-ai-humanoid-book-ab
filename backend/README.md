# Physical AI & Humanoid Robotics Textbook - Backend

This is the backend component of the Physical AI & Humanoid Robotics textbook platform, built with FastAPI. It provides AI services including RAG-powered chatbot, personalization, and Urdu translation features.

## Prerequisites

- Python 3.11+
- Node.js 18+ (for the frontend)
- Docker (for local development of Qdrant and Neon)

## Setup Instructions

### 1. Clone the repository

```bash
git clone <repository-url>
cd physical-ai-humanoid-book1/backend
```

### 2. Create a virtual environment

```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

### 3. Install dependencies

```bash
pip install -r requirements.txt
```

### 4. Set up environment variables

Copy the example environment file and update the values:

```bash
cp .env.example .env
# Edit .env with your configuration
```

### 5. Run the application

```bash
python -m src.api.main
```

The API will be available at `http://localhost:8000`.

## Docker Setup

To run the required services (Qdrant, PostgreSQL) locally with Docker:

```bash
# From the project root directory
docker-compose up -d
```

## API Documentation

Once the server is running, visit `http://localhost:8000/docs` for the interactive API documentation (Swagger UI).