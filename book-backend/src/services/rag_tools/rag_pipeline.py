import os
from typing import List, Dict, Any
from litellm import completion
from rag_tools.embedding_generator import EmbeddingGenerator
from rag_tools.qdrant_manager import QdrantManager
from rag_tools.neon_manager import NeonManager

class RAGPipeline:
    def __init__(self,
                 embedding_generator: EmbeddingGenerator,
                 qdrant_manager: QdrantManager,
                 neon_manager: NeonManager,
                 llm_model_name: str = "gemini/gemini-pro"> # gemini-2.0-flash might be gemini/gemini-pro or another litellm mapping
        
        self.embedding_generator = embedding_generator
        self.qdrant_manager = qdrant_manager
        self.neon_manager = neon_manager
        self.llm_model_name = llm_model_name

        if "GEMINI_API_KEY" not in os.environ:
            print("Warning: GEMINI_API_KEY environment variable not set. LiteLLM might fail.")
        if "NEON_DB_URL" not in os.environ:
            print("Warning: NEON_DB_URL environment variable not set. NeonManager might fail.")
        if "QDRANT_URL" not in os.environ or "QDRANT_API_KEY" not in os.environ:
            print("Warning: QDRANT_URL or QDRANT_API_KEY environment variables not set. QdrantManager might fail.")

    async def generate_answer(self, user_query: str) -> str:
        # 1. Embed user query
        query_embedding = self.embedding_generator.generate_embeddings([user_query])
        if not query_embedding:
            return "Error: Could not generate embedding for the query."
        query_embedding = query_embedding[0]

        # 2. Search Qdrant
        qdrant_results = self.qdrant_manager.search_chunks(query_embedding, limit=5)
        if not qdrant_results:
            return "This detail is not available in the book content."

        # 3. Fetch full text from Neon
        context_texts = []
        for result in qdrant_results:
            neon_chunk = await self.neon_manager.get_chunk_by_qdrant_id(result["qdrant_id"])
            if neon_chunk:
                context_texts.append(neon_chunk.text)
        
        if not context_texts:
            return "This detail is not available in the book content."

        # 4. Assemble context
        context = "\n\n".join(context_texts)
        
        # 5. Call Gemini model
        # The prompt should instruct the LLM to use the provided context ONLY
        messages = [
            {"role": "system", "content": "You are a helpful assistant. Answer the user's question ONLY based on the provided context. If the answer is not in the context, state 'This detail is not available in the book content.' Do not make up information."}, 
            {"role": "user", "content": f"Context: {context}\n\nQuestion: {user_query}"}
        ]

        try:
            response = completion(
                model=self.llm_model_name,
                messages=messages,
                max_tokens=500 # Limit response length
            )
            return response.choices[0].message.content
        except Exception as e:
            print(f"Error calling LLM: {e}")
            return "An error occurred while generating the answer."

if __name__ == '__main__':
    # Set dummy environment variables for testing
    os.environ["GEMINI_API_KEY"] = "YOUR_GEMINI_API_KEY" 
    os.environ["QDRANT_URL"] = "http://localhost:6333" # Replace with your Qdrant instance
    os.environ["QDRANT_API_KEY"] = "your-qdrant-api-key" # Replace with your actual API key
    os.environ["NEON_DB_URL"] = "postgresql://user:password@host:port/database" # Replace with your Neon connection string

    async def run_test():
        try:
            # Initialize managers
            embed_gen = EmbeddingGenerator()
            qdrant_mgr = QdrantManager()
            neon_mgr = NeonManager()
            await neon_mgr.initialize() # Initialize NeonManager for table creation etc.

            # Dummy data to simulate existing data in Qdrant and Neon
            print("Simulating data ingestion...")
            test_chunks = [
                "The primary topic of this book is Physical AI and Humanoid Robotics.",
                "ROS 2 is a middleware for robot control, essential for understanding physical AI.",
                "NVIDIA Isaac Sim is used for photorealistic simulation and synthetic data generation in robotics.",
                "Generative AI models like LLMs can be integrated into robots for conversational capabilities."
            ]
            test_embeddings = embed_gen.generate_embeddings(test_chunks)
            if not test_embeddings:
                print("Failed to generate test embeddings. Exiting.")
                return

            test_metadata = [
                {"module": "Intro", "chapter": "Overview"},
                {"module": "Module 1", "chapter": "ROS2 Fundamentals"},
                {"module": "Module 3", "chapter": "NVIDIA Isaac Platform"},
                {"module": "Module 4", "chapter": "Conversational Robotics"}
            ]

            qdrant_ids = qdrant_mgr.upsert_chunks(test_chunks, test_embeddings, test_metadata)
            
            neon_data = []
            for i, q_id in enumerate(qdrant_ids):
                neon_data.append({
                    "qdrant_id": q_id,
                    "module": test_metadata[i]["module"],
                    "chapter": test_metadata[i]["chapter"],
                    "text": test_chunks[i],
                    "embedding": test_embeddings[i]
                })
            await neon_mgr.insert_chunks(neon_data)
            print("Simulation data ingested.")

            # Initialize RAG Pipeline
            rag_pipeline = RAGPipeline(embed_gen, qdrant_mgr, neon_mgr)

            # Test queries
            query1 = "What is the main subject of this book?"
            print(f"\nUser Query: {query1}")
            answer1 = await rag_pipeline.generate_answer(query1)
            print(f"RAG Answer: {answer1}")

            query2 = "Which tool is used for photorealistic simulation?"
            print(f"\nUser Query: {query2}")
            answer2 = await rag_pipeline.generate_answer(query2)
            print(f"RAG Answer: {answer2}")

            query3 = "What is the capital of France?"
            print(f"\nUser Query: {query3}")
            answer3 = await rag_pipeline.generate_answer(query3)
            print(f"RAG Answer: {answer3}")

        except ValueError as e:
            print(f"Configuration error: {e}")
        except Exception as e:
            print(f"An unexpected error occurred: {e}")
        finally:
            # Clean up dummy environment variables (optional)
            del os.environ["GEMINI_API_KEY"]
            del os.environ["QDRANT_URL"]
            del os.environ["QDRANT_API_KEY"]
            del os.environ["NEON_DB_URL"]

    import asyncio
    asyncio.run(run_test())
