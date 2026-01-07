# API Contract: Content Service

## Overview
The Content service provides access to textbook modules, chapters, and related resources. It supports multilingual content delivery and personalization.

## Base URL
`/api/v1/content`

## Endpoints

### Get all modules
- **GET** `/modules`
- **Description**: Retrieve a list of all available modules
- **Authentication**: Optional (affects personalization)
- **Query Parameters**:
  - `language` (optional): Language code (default: "en", options: ["en", "ur"])
- **Response**:
  - 200 OK
  ```json
  {
    "modules": [
      {
        "id": "string",
        "title": "string",
        "description": "string",
        "order": "integer",
        "learning_objectives": ["string"],
        "estimated_duration": "integer (minutes)",
        "progress": {
          "completed_chapters": "integer",
          "total_chapters": "integer"
        }
      }
    ]
  }
  ```

### Get module details
- **GET** `/modules/{module_id}`
- **Description**: Retrieve detailed information about a specific module
- **Authentication**: Optional (affects personalization)
- **Query Parameters**:
  - `language` (optional): Language code (default: "en", options: ["en", "ur"])
- **Response**:
  - 200 OK
  ```json
  {
    "id": "string",
    "title": "string",
    "description": "string",
    "order": "integer",
    "learning_objectives": ["string"],
    "prerequisites": ["string (module IDs)"],
    "chapters": [
      {
        "id": "string",
        "title": "string",
        "order": "integer",
        "estimated_duration": "integer (minutes)",
        "learning_objectives": ["string"],
        "progress": {
          "status": "string (not-started|in-progress|completed)",
          "completion_percentage": "integer"
        }
      }
    ]
  }
  ```

### Get chapter content
- **GET** `/chapters/{chapter_id}`
- **Description**: Retrieve content for a specific chapter
- **Authentication**: Optional (affects personalization)
- **Query Parameters**:
  - `language` (optional): Language code (default: "en", options: ["en", "ur"])
  - `personalize` (optional): Whether to personalize content (default: true)
- **Response**:
  - 200 OK
  ```json
  {
    "id": "string",
    "module_id": "string",
    "title": "string",
    "content": "string (Markdown format)",
    "learning_objectives": ["string"],
    "estimated_duration": "integer (minutes)",
    "hands_on_lab": {
      "title": "string",
      "description": "string",
      "simulation_environment": "string (gazebo|unity|nvidia-isaac|none)",
      "code_template": "string (Markdown with code)",
      "solution": "string (Markdown with code)"
    },
    "code_examples": [
      {
        "title": "string",
        "code": "string",
        "language": "string (python|bash|etc.)",
        "explanation": "string"
      }
    ],
    "review_questions": [
      {
        "question": "string",
        "type": "string (multiple-choice|short-answer|practical)",
        "options": ["string (for multiple-choice)"]
      }
    ],
    "glossary": [
      {
        "term": "string",
        "definition": "string"
      }
    ]
  }
  ```

### Translate content
- **GET** `/translate/{chapter_id}`
- **Description**: Get translated content for a chapter
- **Authentication**: Optional
- **Query Parameters**:
  - `target_language`: Target language code (required, options: ["ur"])
  - `source_language`: Source language code (optional, default: "en")
- **Response**:
  - 200 OK
  ```json
  {
    "chapter_id": "string",
    "source_language": "string",
    "target_language": "string",
    "title": "string",
    "content": "string (Markdown format)",
    "hands_on_lab": {
      "title": "string",
      "description": "string"
    }
  }
  ```

### Search content
- **GET** `/search`
- **Description**: Search across all content
- **Authentication**: Optional
- **Query Parameters**:
  - `q`: Search query (required)
  - `language` (optional): Language code (default: "en", options: ["en", "ur"])
  - `limit` (optional): Max results to return (default: 10, max: 50)
- **Response**:
  - 200 OK
  ```json
  {
    "query": "string",
    "results": [
      {
        "id": "string",
        "type": "string (module|chapter|section)",
        "title": "string",
        "content_preview": "string",
        "relevance_score": "number",
        "url": "string"
      }
    ]
  }
  ```

## Error Responses

All endpoints may return:
- 400 Bad Request: Invalid request parameters
- 401 Unauthorized: Missing or invalid authentication token (when required)
- 404 Not Found: Module, chapter, or resource not found
- 422 Unprocessable Entity: Content not available in requested language
- 500 Internal Server Error: Server error

## Rate Limiting
- 1000 requests per hour per user
- 429 Too Many Requests if exceeded