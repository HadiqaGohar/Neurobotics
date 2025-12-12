import os
from typing import List
from litellm import embedding


class EmbeddingGenerator:
    def __init__(self, model_name: str = "gemini/gemini-embedding-001"):
        self.model_name = model_name
        # LiteLLM automatically picks up API keys from environment variables
        # e.g., GEMINI_API_KEY for Gemini models.
        # It's good practice to ensure the key is set.
        if "GEMINI_API_KEY" not in os.environ:
            print("Warning: GEMINI_API_KEY environment variable not set. LiteLLM might fail.")

    def generate_embeddings(self, texts: List[str]) -> List[List[float]]:
        if not texts:
            return []

        try:
            # LiteLLM's embedding function
            # The input argument expects a list of strings
            response = embedding(
                model=self.model_name,
                input=texts
            )
            # LiteLLM response structure for embeddings:
            # response.data is a list of Embedding objects
            # Each Embedding object has an 'embedding' attribute which is the list of floats
            return [data.embedding for data in response.data]
        except Exception as e:
            print(f"Error generating embeddings with LiteLLM: {e}")
            # Depending on error handling strategy, might re-raise, log, or return empty
            return []

if __name__ == '__main__':
    # For local testing, ensure GEMINI_API_KEY is set in your environment
    # For example: export GEMINI_API_KEY="YOUR_API_KEY"
    # Or uncomment the line below for temporary testing (NOT recommended for production)
    # os.environ["GEMINI_API_KEY"] = "YOUR_GEMINI_API_KEY"

    try:
        generator = EmbeddingGenerator()

        # Test with a single chunk
        print("Test Case 1: Single chunk")
        single_chunk_text = ["This is a single piece of text to embed."]
        single_embedding = generator.generate_embeddings(single_chunk_text)
        if single_embedding:
            print(f"Single embedding dimension: {len(single_embedding[0])}") # Expected: 1536
            print(f"Single embedding (first 5 values): {single_embedding[0][:5]}\n")
        else:
            print("Failed to generate single embedding.\n")

        # Test with multiple chunks
        print("Test Case 2: Multiple chunks")
        multiple_chunks_texts = [
            "First chunk of information.",
            "Second chunk provides more details.",
            "Third chunk concludes the discussion."
        ]
        multiple_embeddings = generator.generate_embeddings(multiple_chunks_texts)
        if multiple_embeddings:
            print(f"Number of embeddings: {len(multiple_embeddings)}") # Expected: 3
            print(f"First embedding dimension: {len(multiple_embeddings[0])}") # Expected: 1536
            print(f"Second embedding dimension: {len(multiple_embeddings[1])}") # Expected: 1536
            print(f"Third embedding dimension: {len(multiple_embeddings[2])}\n")
        else:
            print("Failed to generate multiple embeddings.\n")

        # Test with empty list
        print("Test Case 3: Empty list")
        empty_embeddings = generator.generate_embeddings([])
        print(f"Empty list embeddings: {empty_embeddings}\n") # Expected: []

    except ValueError as e:
        print(f"Configuration error: {e}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")

    # Clean up dummy API key if set for temporary testing
    # if "YOUR_GEMINI_API_KEY" in os.environ.get("GEMINI_API_KEY", ""):
    #    del os.environ["GEMINI_API_KEY"]
