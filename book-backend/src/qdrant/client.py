from qdrant_client import QdrantClient, models
import os
from dotenv import load_dotenv

load_dotenv()

QDRANT_URL = os.getenv("QDRANT_URL", "http://localhost:6333")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")

class QdrantManager:
    _instance = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(QdrantManager, cls).__new__(cls)
            cls._instance.client = QdrantClient(
                url=QDRANT_URL,
                api_key=QDRANT_API_KEY,
            )
            print(f"Qdrant client initialized for URL: {QDRANT_URL}")
        return cls._instance

    def get_client(self):
        return self.client

    def create_collections(self, embedding_dimension: int):
        # Collection for content variants
        self.client.recreate_collection(
            collection_name="content_variants",
            vectors_config=models.VectorParams(size=embedding_dimension, distance=models.Distance.COSINE),
        )
        print("Qdrant collection 'content_variants' created/recreated.")

        # Collection for user profiles
        self.client.recreate_collection(
            collection_name="user_profiles",
            vectors_config=models.VectorParams(size=embedding_dimension, distance=models.Distance.COSINE),
        )
        print("Qdrant collection 'user_profiles' created/recreated.")

    def upsert_vectors(self, collection_name: str, ids: list, vectors: list, payloads: list):
        self.client.upsert(
            collection_name=collection_name,
            points=models.Batch(
                ids=ids,
                vectors=vectors,
                payloads=payloads,
            ),
        )
        print(f"Upserted {len(ids)} vectors into '{collection_name}' collection.")

    def search_vectors(self, collection_name: str, query_vector: list, limit: int = 5, query_filter: models.Filter = None):
        search_result = self.client.search(
            collection_name=collection_name,
            query_vector=query_vector,
            query_filter=query_filter,
            limit=limit,
        )
        return search_result

# Helper function to get Qdrant client instance
def get_qdrant_manager():
    return QdrantManager()
