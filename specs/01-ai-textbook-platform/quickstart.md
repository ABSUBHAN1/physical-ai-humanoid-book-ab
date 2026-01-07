# Quickstart Guide: AI Textbook Platform

## Prerequisites

- Node.js 18+ (for Docusaurus frontend)
- Python 3.11+ (for FastAPI backend)
- Docker (for local development of Qdrant and Neon)
- Git

## Setting Up the Development Environment

### 1. Clone the Repository

```bash
git clone <repository-url>
cd physical-ai-humanoid-book1
```

### 2. Set Up Backend (FastAPI)

```bash
# Navigate to backend directory
cd backend

# Create virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt

# Set up environment variables
cp .env.example .env
# Edit .env with your configuration
```

### 3. Set Up Frontend (Docusaurus)

```bash
# Navigate to frontend directory
cd frontend

# Install dependencies
npm install

# Set up environment variables
cp .env.example .env
# Edit .env with your backend API URL
```

### 4. Set Up Services

For local development, you can run services using Docker:

```bash
# In the project root
docker-compose up -d
```

This will start:
- Qdrant (vector database for RAG)
- PostgreSQL (Neon-compatible for user data)

## Running the Application

### 1. Start the Backend

```bash
cd backend
source venv/bin/activate  # On Windows: venv\Scripts\activate
python -m src.api.main
```

### 2. Start the Frontend

```bash
cd frontend
npm start
```

The application will be available at `http://localhost:3000`

## Adding Content

### 1. Create a New Module

Create a new directory in the `modules/` folder:

```bash
mkdir modules/03_gazebo_simulation
```

### 2. Add Chapters

Create markdown files in the module directory:

```bash
touch modules/03_gazebo_simulation/01_introduction_to_gazebo.md
touch modules/03_gazebo_simulation/02_gazebo_ros_integration.md
```

### 3. Follow the Chapter Template

Each chapter should follow the required 10-section format:

```markdown
---
title: Introduction to Gazebo
description: Learn the basics of the Gazebo simulation environment
---

## Chapter Overview
[Overview of what the chapter covers]

## Why This Matters in Physical AI
[Explanation of why this topic is important in Physical AI]

## Core Concepts
[Key concepts covered in this chapter]

## System Architecture or Mental Model
[How the system works conceptually]

## Hands-On Lab/Simulation
[Practical exercise for students]

## Code Examples
[Relevant code snippets with explanations]

## Common Mistakes & Debugging
[Common issues and how to resolve them]

## Industry & Research Context
[Real-world applications and research]

## Review Questions
[Questions to test understanding]

## Glossary
[Key terms defined]
```

### 4. Generate Embeddings for RAG

After adding content, generate embeddings for the RAG system:

```bash
cd backend
source venv/bin/activate  # On Windows: venv\Scripts\activate
python -m src.core.embeddings --process-new-content
```

## Key Features

### 1. AI Chatbot
- Accessible on every page via the floating widget
- Context-aware responses based on current chapter/module
- Supports follow-up questions within conversation sessions

### 2. Personalization
- Adjust content difficulty based on user's technical background
- Personalized learning paths
- Progress tracking and recommendations

### 3. Multilingual Support
- Toggle between English and Urdu
- Technical terminology preserved in translations
- Language preference saved per user

### 4. Simulation Integration
- Embedded simulation environments (Gazebo, Unity, NVIDIA Isaac)
- Direct interaction with simulations from textbook
- Code examples that connect to simulations

## API Endpoints

### Chatbot API
- `POST /api/v1/chatbot/query` - Submit a question to the chatbot
- `GET /api/v1/chatbot/session/{session_id}` - Retrieve chat history

### Content API
- `GET /api/v1/content/modules` - List all modules
- `GET /api/v1/content/chapters/{chapter_id}` - Get specific chapter content
- `GET /api/v1/content/translate/{chapter_id}?lang=ur` - Get translated content

### User API
- `POST /api/v1/user/login` - User authentication
- `GET /api/v1/user/profile` - Get user profile and preferences
- `PUT /api/v1/user/preferences` - Update user preferences

## Testing

### Backend Tests
```bash
cd backend
python -m pytest tests/
```

### Frontend Tests
```bash
cd frontend
npm test
```

## Deployment

### GitHub Pages (Frontend)
The frontend is configured for GitHub Pages deployment. The workflow is defined in `.github/workflows/deploy.yml`.

### Backend Deployment
The backend can be deployed to any cloud provider that supports Python applications (AWS, GCP, Azure, etc.).

## Troubleshooting

### Common Issues

1. **Chatbot not responding**: Check that the backend is running and the API endpoint is accessible
2. **Content not appearing**: Verify that the markdown files follow the correct format and are in the right directory
3. **Translation not working**: Ensure the translation API keys are properly configured
4. **Simulation not loading**: Check that the simulation environment is properly configured and accessible