#!/usr/bin/env python3
"""
Script to run comprehensive load tests for the RAG chatbot system.
"""

import subprocess
import sys
import time
import json
import os
from pathlib import Path


def run_benchmark_tests():
    """Run pytest-benchmark tests."""
    print("🔄 Running benchmark tests...")
    
    cmd = [
        "uv", "run", "pytest", 
        "tests/load/benchmark_tests.py",
        "--benchmark-only",
        "--benchmark-json=tests/load/benchmark_results.json",
        "--benchmark-sort=mean",
        "-v"
    ]
    
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, cwd="book-backend")
        
        if result.returncode == 0:
            print("✅ Benchmark tests completed successfully")
            print(result.stdout)
            
            # Parse and display key metrics
            if os.path.exists("book-backend/tests/load/benchmark_results.json"):
                with open("book-backend/tests/load/benchmark_results.json", "r") as f:
                    data = json.load(f)
                    
                print("\n📊 Key Performance Metrics:")
                for benchmark in data.get("benchmarks", []):
                    name = benchmark["name"]
                    mean = benchmark["stats"]["mean"]
                    stddev = benchmark["stats"]["stddev"]
                    print(f"  {name}: {mean:.4f}s ± {stddev:.4f}s")
        else:
            print("❌ Benchmark tests failed")
            print(result.stderr)
            return False
            
    except Exception as e:
        print(f"❌ Error running benchmark tests: {e}")
        return False
    
    return True


def run_locust_tests():
    """Run Locust load tests."""
    print("\n🔄 Starting Locust load tests...")
    print("Note: Make sure your FastAPI server is running on http://localhost:8000")
    
    # Check if server is running
    try:
        import requests
        response = requests.get("http://localhost:8000/health", timeout=5)
        if response.status_code != 200:
            print("❌ Server not responding at http://localhost:8000")
            return False
    except Exception:
        print("❌ Cannot connect to server at http://localhost:8000")
        print("Please start the server with: cd book-backend && uv run uvicorn main:app --reload")
        return False
    
    print("✅ Server is running, starting load tests...")
    
    # Run Locust in headless mode for automated testing
    cmd = [
        "uv", "run", "locust",
        "-f", "tests/load/locustfile.py",
        "--host=http://localhost:8000",
        "--users=10",
        "--spawn-rate=2",
        "--run-time=60s",
        "--headless",
        "--html=tests/load/locust_report.html"
    ]
    
    try:
        result = subprocess.run(cmd, cwd="book-backend", timeout=120)
        
        if result.returncode == 0:
            print("✅ Locust load tests completed successfully")
            print("📊 Report saved to: book-backend/tests/load/locust_report.html")
        else:
            print("❌ Locust load tests failed")
            return False
            
    except subprocess.TimeoutExpired:
        print("⏰ Load tests completed (timeout reached)")
    except Exception as e:
        print(f"❌ Error running Locust tests: {e}")
        return False
    
    return True


def run_interactive_locust():
    """Run Locust with web UI for interactive testing."""
    print("\n🌐 Starting Locust with web UI...")
    print("Access the web interface at: http://localhost:8089")
    
    cmd = [
        "uv", "run", "locust",
        "-f", "tests/load/locustfile.py",
        "--host=http://localhost:8000"
    ]
    
    try:
        subprocess.run(cmd, cwd="book-backend")
    except KeyboardInterrupt:
        print("\n👋 Locust web UI stopped")


def main():
    """Main function to run load tests."""
    print("🚀 RAG Chatbot Load Testing Suite")
    print("=" * 40)
    
    if len(sys.argv) > 1:
        test_type = sys.argv[1].lower()
        
        if test_type == "benchmark":
            run_benchmark_tests()
        elif test_type == "locust":
            run_locust_tests()
        elif test_type == "interactive":
            run_interactive_locust()
        else:
            print("❌ Invalid test type. Use: benchmark, locust, or interactive")
            sys.exit(1)
    else:
        # Run all tests
        print("Running all load tests...\n")
        
        # Run benchmark tests first
        if not run_benchmark_tests():
            print("❌ Benchmark tests failed, skipping Locust tests")
            sys.exit(1)
        
        # Run Locust tests
        if not run_locust_tests():
            print("❌ Locust tests failed")
            sys.exit(1)
        
        print("\n🎉 All load tests completed successfully!")
        print("\nResults:")
        print("  - Benchmark results: book-backend/tests/load/benchmark_results.json")
        print("  - Locust report: book-backend/tests/load/locust_report.html")


if __name__ == "__main__":
    main()