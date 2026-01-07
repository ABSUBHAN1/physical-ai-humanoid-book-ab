"""
Basic test to verify the implementation of the Physical AI & Humanoid Robotics textbook platform.
"""
import os
import sys
from pathlib import Path

# Add the backend/src directory to the path so we can import modules
sys.path.insert(0, str(Path(__file__).parent / "backend" / "src"))

def test_project_structure():
    """Test that the basic project structure is in place."""
    required_paths = [
        "backend/src/models/__init__.py",
        "backend/src/services/rag_service.py",
        "backend/src/services/content_service.py",
        "backend/src/api/v1/chatbot.py",
        "backend/src/api/v1/content.py",
        "backend/src/core/embeddings.py",
        "frontend/src/components/ChatbotWidget/ChatbotWidget.js",
        "frontend/src/components/SimulationEmbed/SimulationEmbed.js",
        "frontend/src/theme/DocPage/index.js",
        "modules/01_introduction/01_what_is_physical_ai.md",
        "modules/01_introduction/examples/simple_physical_ai_agent.py"
    ]
    
    missing_paths = []
    for path in required_paths:
        full_path = Path(__file__).parent / path
        if not full_path.exists():
            missing_paths.append(path)
    
    if missing_paths:
        print(f"Missing required paths: {missing_paths}")
        return False

    print("[PASS] All required paths exist")
    return True

def test_content_structure():
    """Test that the content follows the required 10-section format."""
    content_path = Path(__file__).parent / "modules/01_introduction/01_what_is_physical_ai.md"

    if not content_path.exists():
        print("Content file does not exist")
        return False
    
    content = content_path.read_text()
    
    required_sections = [
        "Chapter Overview",
        "Why This Matters in Physical AI", 
        "Core Concepts",
        "System Architecture or Mental Model",
        "Hands-On Lab/Simulation",
        "Code Examples",
        "Common Mistakes & Debugging",
        "Industry & Research Context",
        "Review Questions",
        "Glossary"
    ]
    
    missing_sections = []
    for section in required_sections:
        if f"## {section}" not in content:
            missing_sections.append(section)
    
    if missing_sections:
        print(f"Missing required sections: {missing_sections}")
        return False

    print("[PASS] Content follows the required 10-section format")
    return True

def main():
    """Run all tests."""
    print("Testing Physical AI & Humanoid Robotics textbook implementation...")
    print()

    tests = [
        test_project_structure,
        test_content_structure
    ]

    all_passed = True
    for test in tests:
        if not test():
            all_passed = False
        print()

    if all_passed:
        print("[PASS] All tests passed!")
        return 0
    else:
        print("[FAIL] Some tests failed!")
        return 1

if __name__ == "__main__":
    sys.exit(main())