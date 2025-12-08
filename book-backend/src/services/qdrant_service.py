from typing import List, Dict, Any
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct
import uuid

COLLECTION_NAME = "book_content"
VECTOR_DIMENSION = 384  # Dimension for 'all-MiniLM-L6-v2'


class QdrantService:
    def __init__(self, qdrant_client: QdrantClient):
        self.client = qdrant_client
        self.collection_name = COLLECTION_NAME
        self.vector_dimension = VECTOR_DIMENSION

    def create_collection(self, collection_name: str = None):
        """Create or recreate a Qdrant collection"""
        collection_name = collection_name or self.collection_name
        
        try:
            self.client.recreate_collection(
                collection_name=collection_name,
                vectors_config=VectorParams(size=self.vector_dimension, distance=Distance.COSINE),
            )
            print(f"Collection '{collection_name}' created successfully.")
        except Exception as e:
            print(f"Error creating collection '{collection_name}': {e}")
            raise

    def collection_exists(self, collection_name: str = None) -> bool:
        """Check if a collection exists"""
        collection_name = collection_name or self.collection_name
        
        try:
            collections = self.client.get_collections()
            return any(col.name == collection_name for col in collections.collections)
        except Exception as e:
            print(f"Error checking collection existence: {e}")
            return False

    def ensure_collection_exists(self, collection_name: str = None):
        """Ensure collection exists, create if it doesn't"""
        collection_name = collection_name or self.collection_name
        
        if not self.collection_exists(collection_name):
            self.create_collection(collection_name)

    async def add_vector(self, collection_name: str, vector: List[float], payload: Dict[str, Any], point_id: str = None):
        """Add a single vector to the collection"""
        collection_name = collection_name or self.collection_name
        point_id = point_id or str(uuid.uuid4())
        
        try:
            self.ensure_collection_exists(collection_name)
            
            point = PointStruct(
                id=point_id,
                vector=vector,
                payload=payload
            )
            
            self.client.upsert(
                collection_name=collection_name,
                points=[point]
            )
            
            return point_id
        except Exception as e:
            print(f"Error adding vector to collection '{collection_name}': {e}")
            raise

    async def add_vectors_batch(self, collection_name: str, vectors: List[List[float]], payloads: List[Dict[str, Any]], point_ids: List[str] = None):
        """Add multiple vectors to the collection in batch"""
        collection_name = collection_name or self.collection_name
        
        if point_ids is None:
            point_ids = [str(uuid.uuid4()) for _ in vectors]
        
        if len(vectors) != len(payloads) or len(vectors) != len(point_ids):
            raise ValueError("Vectors, payloads, and point_ids must have the same length")
        
        try:
            self.ensure_collection_exists(collection_name)
            
            points = [
                PointStruct(id=point_id, vector=vector, payload=payload)
                for point_id, vector, payload in zip(point_ids, vectors, payloads)
            ]
            
            self.client.upsert(
                collection_name=collection_name,
                points=points
            )
            
            return point_ids
        except Exception as e:
            print(f"Error adding vectors batch to collection '{collection_name}': {e}")
            raise

    def search_vectors(self, collection_name: str, query_vector: List[float], limit: int = 5, score_threshold: float = None):
        """Search for similar vectors"""
        collection_name = collection_name or self.collection_name
        
        try:
            search_params = {
                "collection_name": collection_name,
                "query_vector": query_vector,
                "limit": limit
            }
            
            if score_threshold is not None:
                search_params["score_threshold"] = score_threshold
            
            results = self.client.search(**search_params)
            return results
        except Exception as e:
            print(f"Error searching vectors in collection '{collection_name}': {e}")
            raise

    def delete_vectors(self, collection_name: str, point_ids: List[str]):
        """Delete vectors by their IDs"""
        collection_name = collection_name or self.collection_name
        
        try:
            self.client.delete(
                collection_name=collection_name,
                points_selector=point_ids
            )
        except Exception as e:
            print(f"Error deleting vectors from collection '{collection_name}': {e}")
            raise

    def get_collection_info(self, collection_name: str = None):
        """Get information about a collection"""
        collection_name = collection_name or self.collection_name
        
        try:
            return self.client.get_collection(collection_name)
        except Exception as e:
            print(f"Error getting collection info for '{collection_name}': {e}")
            raise


# Legacy function for backward compatibility
def create_qdrant_collection():
    """Legacy function - use QdrantService class instead"""
    import os
    from qdrant_client import QdrantClient
    
    qdrant_url = os.getenv("QDRANT_URL")
    qdrant_api_key = os.getenv("QDRANT_API_KEY")
    
    if not qdrant_url or not qdrant_api_key:
        raise ValueError("QDRANT_URL or QDRANT_API_KEY environment variable not set")
    
    client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)
    service = QdrantService(client)
    service.create_collection()


if __name__ == "__main__":
    create_qdrant_collection()
