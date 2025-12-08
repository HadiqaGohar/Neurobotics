# Load Testing for RAG Chatbot

This directory contains load testing and performance benchmarking tools for the RAG chatbot system.

## Test Types

### 1. Benchmark Tests (`benchmark_tests.py`)
- Uses `pytest-benchmark` for micro-benchmarks
- Tests individual component performance
- Measures response times and throughput
- Generates detailed performance metrics

### 2. Load Tests (`locustfile.py`)
- Uses Locust for load testing
- Simulates multiple concurrent users
- Tests API endpoints under load
- Generates HTML reports with statistics

## Running Tests

### Quick Start
```bash
# Run all tests
cd book-backend
python tests/load/run_load_tests.py

# Run only benchmark tests
python tests/load/run_load_tests.py benchmark

# Run only Locust tests (requires server running)
python tests/load/run_load_tests.py locust

# Run Locust with web UI
python tests/load/run_load_tests.py interactive
```

### Manual Execution

#### Benchmark Tests
```bash
cd book-backend
uv run pytest tests/load/benchmark_tests.py --benchmark-only --benchmark-sort=mean -v
```

#### Locust Load Tests
```bash
# Start the server first
cd book-backend
uv run uvicorn main:app --reload

# In another terminal, run Locust
cd book-backend
uv run locust -f tests/load/locustfile.py --host=http://localhost:8000
```

## Performance Targets

### Response Time Targets
- Context retrieval: < 500ms
- Embedding generation: < 200ms
- RAG response generation: < 2s
- File upload processing: < 5s

### Throughput Targets
- Chat requests: > 50 req/s
- Concurrent users: > 100 users
- File uploads: > 10 req/s

### Resource Usage Targets
- Memory usage: < 2GB
- CPU usage: < 80%
- Database connections: < 50

## Test Scenarios

### Locust Test Scenarios

1. **RAGChatbotUser** (Weight: 5)
   - Tests `/chat/rag` endpoint
   - Tests `/chat` endpoint for comparison
   - Maintains session state
   - Uses realistic queries

2. **FileUploadUser** (Weight: 1)
   - Tests `/chat/file` endpoint
   - Uploads text documents
   - Processes file content

3. **VoiceChatUser** (Weight: 1)
   - Tests `/chat/voice` endpoint
   - Simulates audio processing
   - Uses mock audio data

### Benchmark Test Categories

1. **RAG Service Benchmarks**
   - Context retrieval performance
   - Response generation speed
   - Memory usage patterns

2. **Embedding Service Benchmarks**
   - Single text embedding
   - Batch embedding processing
   - Document chunking speed

3. **Database Benchmarks**
   - Message insertion speed
   - Session creation performance
   - Query execution time

## Interpreting Results

### Benchmark Results
- Look for `benchmark_results.json` for detailed metrics
- Focus on mean response times and standard deviation
- Compare against performance targets

### Locust Results
- Check `locust_report.html` for comprehensive analysis
- Monitor failure rates and response time percentiles
- Analyze concurrent user capacity

## Troubleshooting

### Common Issues

1. **Server Not Running**
   ```bash
   cd book-backend
   uv run uvicorn main:app --reload
   ```

2. **Dependencies Missing**
   ```bash
   cd book-backend
   uv sync
   ```

3. **Database Connection Issues**
   - Check `.env` file configuration
   - Ensure Neon database is accessible
   - Verify Qdrant connection

4. **Memory Issues During Load Tests**
   - Reduce concurrent users in Locust
   - Monitor system resources
   - Check for memory leaks

### Performance Optimization Tips

1. **Database Optimization**
   - Add indexes for frequently queried fields
   - Use connection pooling
   - Optimize query patterns

2. **Vector Database Optimization**
   - Tune Qdrant collection parameters
   - Optimize embedding dimensions
   - Use appropriate distance metrics

3. **API Optimization**
   - Implement response caching
   - Use async/await properly
   - Optimize serialization

4. **Resource Management**
   - Monitor memory usage
   - Implement proper cleanup
   - Use connection limits