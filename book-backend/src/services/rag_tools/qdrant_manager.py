import os
from qdrant_client import QdrantClient, models
from typing import List, Dict, Any
import uuid

class QdrantManager:
    def __init__(self,
                 url: str = os.getenv("QDRANT_URL"),
                 api_key: str = os.getenv("QDRANT_API_KEY"),
                 collection_name: str = "book_chunks",
                 vector_size: int = 1536,
                 distance_metric: models.Distance = models.Distance.COSINE):
        if not url or not api_key:
            raise ValueError("QDRANT_URL and QDRANT_API_KEY environment variables must be set.")
        
        self.client = QdrantClient(url=url, api_key=api_key)
        self.collection_name = collection_name
        self.vector_size = vector_size
        self.distance_metric = distance_metric
        self._ensure_collection_exists()

    def _ensure_collection_exists(self):
        # Check if collection exists, create if not
        if not self.client.collection_exists(collection_name=self.collection_name):
            print(f"Collection '{self.collection_name}' not found. Creating it...")
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(size=self.vector_size, distance=self.distance_metric),
            )
            print(f"Collection '{self.collection_name}' created successfully.")
        else:
            print(f"Collection '{self.collection_name}' already exists.")

    def upsert_chunks(self,
                      chunks: List[str],
                      embeddings: List[List[float]],
                      metadata: List[Dict[str, str]]) -> List[str]:
        if not chunks or not embeddings or len(chunks) != len(embeddings) or len(chunks) != len(metadata):
            raise ValueError("Chunks, embeddings, and metadata lists must be non-empty and of equal length.")

        points = []
        qdrant_ids = []
        for i, chunk_text in enumerate(chunks):
            chunk_id = str(uuid.uuid4())
            qdrant_ids.append(chunk_id)
            points.append(
                models.PointStruct(
                    id=chunk_id,
                    vector=embeddings[i],
                    payload={
                        "text": chunk_text,
                        "module": metadata[i]["module"],
                        "chapter": metadata[i]["chapter"]
                    }
                )
            )
        
        # Batch upsert points
        response = self.client.upsert(
            collection_name=self.collection_name,
            wait=True,
            points=points
        )
        print(f"Upserted {len(points)} points to Qdrant. Status: {response.status}")
        return qdrant_ids

    def search_chunks(self, query_embedding: List[float], limit: int = 5) -> List[Dict[str, Any]]:
        """
        Searches the Qdrant collection for chunks similar to the query embedding.
        Returns a list of dictionaries, each containing 'qdrant_id', 'score', and 'payload'.
        """
        if not query_embedding:
            return []

        search_result = self.client.search(
            collection_name=self.collection_name,
            query_vector=query_embedding,
            limit=limit,
            with_payload=True # Include the payload (text, module, chapter) in the result
        )
        
        results = []
        for hit in search_result:
            results.append({
                "qdrant_id": hit.id,
                "score": hit.score,
                "text": hit.payload.get("text"),
                "module": hit.payload.get("module"),
                "chapter": hit.payload.get("chapter")
            })
        return results


if __name__ == '__main__':
    # Set dummy environment variables for testing
    os.environ["QDRANT_URL"] = "http://localhost:6333" # Replace with your Qdrant instance
    os.environ["QDRANT_API_KEY"] = "your-qdrant-api-key" # Replace with your actual API key

    try:
        manager = QdrantManager()

        # Dummy data for testing
        test_chunks = [
            "This is the first test chunk about AI.",
            "The second test chunk discusses robotics.",
            "Third chunk covers machine learning concepts."
        ]
        test_embeddings = [
            [0.1] * 1536, # Dummy embedding 1
            [0.2] * 1536, # Dummy embedding 2
            [0.3] * 1536  # Dummy embedding 3
        ]
        test_metadata = [
            {"module": "Module A", "chapter": "Chapter 1"},
            {"module": "Module A", "chapter": "Chapter 2"},
            {"module": "Module B", "chapter": "Chapter 1"}
        ]

        # Test upsert
        print("\nTesting upsert_chunks...")
        ids = manager.upsert_chunks(test_chunks, test_embeddings, test_metadata)
        print(f"Upserted Qdrant IDs: {ids}")

        # Test search
        print("\nTesting search_chunks...")
        query_embedding = [0.15] * 1536 # A dummy query embedding
        search_results = manager.search_chunks(query_embedding, limit=2)
        print(f"Search Results ({len(search_results)} hits):")
        for res in search_results:
            print(f"  ID: {res['qdrant_id']}, Score: {res['score']:.4f}, Module: {res['module']}, Chapter: {res['chapter']}")
            print(f"  Text: {res['text'][:50]}...\n")


    except ValueError as e:
        print(f"Configuration error: {e}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")

    # Clean up dummy environment variables (optional)
    del os.environ["QDRANT_URL"]
    del os.environ["QDRANT_API_KEY"]
