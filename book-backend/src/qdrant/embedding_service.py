import os
from typing import List, Dict, Any

from sentence_transformers import SentenceTransformer 


class EmbeddingService:
    _instance = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(EmbeddingService, cls).__new__(cls)
            # Initialize embedding model here
            cls._instance.model = SentenceTransformer(os.getenv("EMBEDDING_MODEL_NAME", "all-MiniLM-L6-v2"))
            print("EmbeddingService initialized with SentenceTransformer model 'all-MiniLM-L6-v2'.")
        return cls._instance

    # Placeholder for actual embedding generation
    async def generate_embeddings(self, texts: List[str]) -> List[List[float]]:
        """
        Generates embeddings for a list of texts using the configured embedding model.
        """
        print(f"Generating embeddings for {len(texts)} texts using {self._instance.model.model_name_or_path}...")
        # Use the loaded SentenceTransformer model to generate embeddings
        embeddings = self._instance.model.encode(texts)
        return embeddings.tolist() # Convert numpy array to list of lists

    async def get_user_profile_embedding(self, user_preferences: Dict[str, Any]) -> List[float]:
        """
        Generates an embedding for a user's profile based on their preferences.
        This would combine software_background, hardware_background, etc., into a single text or structured input.
        """
        # Example: concatenate relevant preference fields into a single string
        software_info = user_preferences.get("software_background", {})
        hardware_info = user_preferences.get("hardware_background", {})
        content_complexity = user_preferences.get("content_complexity", "moderate")
        explanation_depth = user_preferences.get("explanation_depth", "standard")
        example_style = user_preferences.get("example_style", "practical")

        user_text = (
            f"Software background: {software_info.get('categories', [])}, "
            f"experience level: {software_info.get('experience_level', 'unknown')}, "
            f"languages: {software_info.get('preferred_languages', [])}, "
            f"frameworks: {software_info.get('frameworks', [])}. "
            f"Hardware background: {hardware_info.get('categories', [])}, "
            f"experience level: {hardware_info.get('experience_level', 'unknown')}, "
            f"platforms: {hardware_info.get('platforms', [])}, "
            f"components: {hardware_info.get('components', [])}. "
            f"Prefers content complexity: {content_complexity}, "
            f"explanation depth: {explanation_depth}, "
            f"example style: {example_style}."
        )
        # Generate embedding for the combined text
        embeddings = await self.generate_embeddings([user_text])
        return embeddings[0]

    async def get_content_variant_embedding(self, content_variant_data: Dict[str, Any]) -> List[float]:
        """
        Generates an embedding for a content variant based on its content and metadata.
        """
        content_text = content_variant_data.get("content", {}).get("text", "")
        metadata_text = str(content_variant_data.get("metadata", {})) # Convert metadata JSON to string
        variant_type = content_variant_data.get("variant_type", "")
        chapter_id = content_variant_data.get("chapter_id", "")
        section_id = content_variant_data.get("section_id", "")

        combined_text = (
            f"Chapter: {chapter_id}, Section: {section_id}, Variant Type: {variant_type}. "
            f"Content: {content_text}. Metadata: {metadata_text}"
        )
        # Generate embedding for the combined text
        embeddings = await self.generate_embeddings([combined_text])
        return embeddings[0]

def get_embedding_service():
    return EmbeddingService()