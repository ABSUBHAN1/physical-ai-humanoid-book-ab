# API Contract: Chatbot Service

## Overview
The Chatbot service provides AI-powered question answering capabilities using RAG (Retrieval-Augmented Generation). It retrieves relevant content from the textbook and generates contextual responses.

## Base URL
`/api/v1/chatbot`

## Endpoints

### Create a new chat session
- **POST** `/session`
- **Description**: Creates a new chat session for a user
- **Authentication**: Required (JWT token)
- **Request Body**:
  ```json
  {
    "module_id": "string (optional)",
    "chapter_id": "string (optional)"
  }
  ```
- **Response**:
  - 201 Created
  ```json
  {
    "session_id": "string",
    "created_at": "ISO 8601 datetime"
  }
  ```

### Submit a query to the chatbot
- **POST** `/query`
- **Description**: Submit a question to the chatbot and receive a response
- **Authentication**: Required (JWT token)
- **Request Body**:
  ```json
  {
    "session_id": "string",
    "query": "string (max 1000 characters)",
    "context": {
      "module_id": "string (optional)",
      "chapter_id": "string (optional)"
    }
  }
  ```
- **Response**:
  - 200 OK
  ```json
  {
    "response": "string",
    "sources": ["string (content IDs)"],
    "followup_questions": ["string (suggested followup questions)"]
  }
  ```

### Get chat session history
- **GET** `/session/{session_id}`
- **Description**: Retrieve the history of messages in a chat session
- **Authentication**: Required (JWT token)
- **Response**:
  - 200 OK
  ```json
  {
    "session_id": "string",
    "messages": [
      {
        "id": "string",
        "role": "string (user|assistant)",
        "content": "string",
        "timestamp": "ISO 8601 datetime"
      }
    ]
  }
  ```

## Error Responses

All endpoints may return:
- 400 Bad Request: Invalid request parameters
- 401 Unauthorized: Missing or invalid authentication token
- 404 Not Found: Session or resource not found
- 500 Internal Server Error: Server error

## Rate Limiting
- 100 requests per hour per user
- 429 Too Many Requests if exceeded