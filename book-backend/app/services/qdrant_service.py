"""
Qdrant client connection and collection management service
"""

from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, CollectionStatus
from qdrant_client.http import models
from app.core.config import get_settings
import logging
from typing import List, Dict, Any, Optional

logger = logging.getLogger(__name__)

# Get settings
settings = get_settings()


class QdrantService:
    """Service for managing Qdrant vector database operations"""
    
    def __init__(self):
        self.client = None
        self.collection_name = "book_content"
        self.vector_size = 384  # all-MiniLM-L6-v2 embedding size
        self._initialize_client()
    
    def _initialize_client(self):
        """Initialize Qdrant client connection"""
        try:
            if not settings.qdrant_url or not settings.qdrant_api_key:
                logger.warning("Qdrant credentials not configured")
                return
            
            self.client = QdrantClient(
                url=settings.qdrant_url,
                api_key=settings.qdrant_api_key,
                timeout=30
            )
            
            logger.info("Qdrant client initialized successfully")
            
            # Initialize collection if it doesn't exist
            self._ensure_collection_exists()
            
        except Exception as e:
            logger.error(f"Failed to initialize Qdrant client: {e}")
            self.client = None
    
    def _ensure_collection_exists(self):
        """Ensure the book content collection exists"""
        try:
            if not self.client:
                return
            
            # Check if collection exists
            collections = self.client.get_collections()
            collection_names = [col.name for col in collections.collections]
            
            if self.collection_name not in collection_names:
                # Create collection
                self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=VectorParams(
                        size=self.vector_size,
                        distance=Distance.COSINE
                    )
                )
                logger.info(f"Created Qdrant collection: {self.collection_name}")
            else:
                logger.info(f"Qdrant collection already exists: {self.collection_name}")
                
        except Exception as e:
            logger.error(f"Error ensuring collection exists: {e}")
    
    def get_client(self) -> Optional[QdrantClient]:
        """Get Qdrant client instance"""
        return self.client
    
    def is_connected(self) -> bool:
        """Check if Qdrant client is connected"""
        try:
            if not self.client:
                return False
            
            # Try to get collections as a health check
            self.client.get_collections()
            return True
            
        except Exception as e:
            logger.error(f"Qdrant connection check failed: {e}")
            return False
    
    def get_collection_info(self) -> Dict[str, Any]:
        """Get information about the book content collection"""
        try:
            if not self.client:
                return {"error": "Qdrant client not initialized"}
            
            collection_info = self.client.get_collection(self.collection_name)
            
            return {
                "name": self.collection_name,
                "status": collection_info.status,
                "vectors_count": collection_info.vectors_count,
                "indexed_vectors_count": collection_info.indexed_vectors_count,
                "points_count": collection_info.points_count,
                "segments_count": collection_info.segments_count,
                "config": {
                    "vector_size": self.vector_size,
                    "distance": "COSINE"
                }
            }
            
        except Exception as e:
            logger.error(f"Error getting collection info: {e}")
            return {"error": str(e)}
    
    def search_vectors(
        self, 
        query_vector: List[float], 
        limit: int = 5,
        score_threshold: float = 0.0
    ) -> List[Dict[str, Any]]:
        """Search for similar vectors in the collection"""
        try:
            if not self.client:
                logger.error("Qdrant client not initialized")
                return []
            
            search_result = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_vector,
                limit=limit,
                score_threshold=score_threshold,
                with_payload=True,
                with_vectors=False
            )
            
            results = []
            for hit in search_result:
                results.append({
                    "id": hit.id,
                    "score": hit.score,
                    "payload": hit.payload
                })
            
            return results
            
        except Exception as e:
            logger.error(f"Error searching vectors: {e}")
            return []
    
    def upsert_vectors(
        self, 
        points: List[Dict[str, Any]]
    ) -> bool:
        """Insert or update vectors in the collection"""
        try:
            if not self.client:
                logger.error("Qdrant client not initialized")
                return False
            
            # Convert points to Qdrant format
            qdrant_points = []
            for point in points:
                qdrant_points.append(
                    models.PointStruct(
                        id=point["id"],
                        vector=point["vector"],
                        payload=point.get("payload", {})
                    )
                )
            
            # Upsert points
            operation_info = self.client.upsert(
                collection_name=self.collection_name,
                points=qdrant_points
            )
            
            logger.info(f"Upserted {len(points)} vectors to Qdrant")
            return operation_info.status == models.UpdateStatus.COMPLETED
            
        except Exception as e:
            logger.error(f"Error upserting vectors: {e}")
            return False
    
    def delete_vectors(self, point_ids: List[str]) -> bool:
        """Delete vectors from the collection"""
        try:
            if not self.client:
                logger.error("Qdrant client not initialized")
                return False
            
            operation_info = self.client.delete(
                collection_name=self.collection_name,
                points_selector=models.PointIdsList(
                    points=point_ids
                )
            )
            
            logger.info(f"Deleted {len(point_ids)} vectors from Qdrant")
            return operation_info.status == models.UpdateStatus.COMPLETED
            
        except Exception as e:
            logger.error(f"Error deleting vectors: {e}")
            return False
    
    def clear_collection(self) -> bool:
        """Clear all vectors from the collection"""
        try:
            if not self.client:
                logger.error("Qdrant client not initialized")
                return False
            
            # Delete collection and recreate it
            self.client.delete_collection(self.collection_name)
            
            # Recreate collection
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=VectorParams(
                    size=self.vector_size,
                    distance=Distance.COSINE
                )
            )
            
            logger.info(f"Cleared collection: {self.collection_name}")
            return True
            
        except Exception as e:
            logger.error(f"Error clearing collection: {e}")
            return False
    
    def get_vector_by_id(self, point_id: str) -> Optional[Dict[str, Any]]:
        """Get a specific vector by ID"""
        try:
            if not self.client:
                return None
            
            points = self.client.retrieve(
                collection_name=self.collection_name,
                ids=[point_id],
                with_payload=True,
                with_vectors=True
            )
            
            if points:
                point = points[0]
                return {
                    "id": point.id,
                    "vector": point.vector,
                    "payload": point.payload
                }
            
            return None
            
        except Exception as e:
            logger.error(f"Error getting vector by ID {point_id}: {e}")
            return None


# Global Qdrant service instance
qdrant_service = QdrantService()