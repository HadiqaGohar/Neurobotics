#!/bin/bash

# Chatbot Development Startup Script
echo "🤖 Starting Chatbot Development Environment..."

# Function to check if a command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Check dependencies
echo "📋 Checking dependencies..."

if ! command_exists uv; then
    echo "❌ uv is not installed. Please install it first: https://docs.astral.sh/uv/getting-started/installation/"
    exit 1
fi

if ! command_exists npm; then
    echo "❌ npm is not installed. Please install Node.js and npm first."
    exit 1
fi

# Check if .env file exists
if [ ! -f "book-backend/.env" ]; then
    echo "⚠️  .env file not found in book-backend/"
    echo "Please create book-backend/.env with your GEMINI_API_KEY"
    echo "Example:"
    echo "GEMINI_API_KEY=your_api_key_here"
    exit 1
fi

echo "✅ Dependencies check passed!"

# Start backend
echo "🚀 Starting backend server..."
cd book-backend
uv sync
uv run python main.py &
BACKEND_PID=$!
cd ..

# Wait a moment for backend to start
sleep 3

# Start frontend
echo "🌐 Starting frontend server..."
cd book-frontend
npm install
npm start &
FRONTEND_PID=$!
cd ..

echo ""
echo "🎉 Chatbot environment started successfully!"
echo ""
echo "📍 Services:"
echo "   Backend API: http://localhost:8000"
echo "   Frontend:    http://localhost:3000"
echo ""
echo "🤖 The floating chatbot should appear in the bottom-right corner"
echo ""
echo "Press Ctrl+C to stop all services"

# Function to cleanup on exit
cleanup() {
    echo ""
    echo "🛑 Stopping services..."
    kill $BACKEND_PID 2>/dev/null
    kill $FRONTEND_PID 2>/dev/null
    echo "✅ All services stopped"
    exit 0
}

# Set trap to cleanup on script exit
trap cleanup INT TERM

# Wait for user to stop
wait