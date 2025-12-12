class TextChunker:
    def __init__(self, chunk_size: int = 1500, overlap: int = 200):
        if not (0 <= overlap < chunk_size):
            raise ValueError("Overlap must be less than chunk_size and non-negative.")
        self.chunk_size = chunk_size
        self.overlap = overlap

    def chunk_text(self, text: str):
        chunks = []
        if not text:
            return chunks

        start_index = 0
        while start_index < len(text):
            end_index = min(start_index + self.chunk_size, len(text))
            chunk = text[start_index:end_index]
            chunks.append(chunk)

            if end_index == len(text):
                break
            
            start_index += (self.chunk_size - self.overlap)
            # Ensure start_index doesn't go beyond text length if overlap is too large for remaining text
            if start_index >= len(text):
                break

        return chunks

if __name__ == '__main__':
    chunker = TextChunker() 
    
    # Test case 1: Empty text
    print("Test Case 1: Empty Text")
    empty_chunks = chunker.chunk_text("")
    print(f"Chunks: {empty_chunks}\n") # Expected: []

    # Test case 2: Text shorter than chunk_size
    print("Test Case 2: Text shorter than chunk_size")
    short_text = "This is a short text."
    short_chunks = chunker.chunk_text(short_text)
    print(f"Chunks: {short_chunks}\n") # Expected: ["This is a short text."]

    # Test case 3: Text exactly chunk_size
    print("Test Case 3: Text exactly chunk_size")
    exact_text = "a" * 1500
    exact_chunks = chunker.chunk_text(exact_text)
    print(f"Chunks: {exact_chunks}\n") # Expected: ["a" * 1500]

    # Test case 4: Text slightly longer than chunk_size, one overlap
    print("Test Case 4: Text slightly longer than chunk_size")
    long_text = "A" * 1500 + "B" * 300
    long_chunks = chunker.chunk_text(long_text)
    print(f"Number of chunks: {len(long_chunks)}") # Expected: 2
    print(f"Chunk 1 length: {len(long_chunks[0])}") # Expected: 1500 
    print(f"Chunk 2 length: {len(long_chunks[1])}\n") # Expected: 1500 - 200 + 300 = 1600 (incorrect logic, should be 300, or 200 + 300 if it starts at 1300)
    # The actual second chunk starts at 1500 - 200 = 1300, and goes to 1500+300=1800 if text is long enough
    # If text is 1800, first chunk is [0:1500], next start is 1300. next chunk is [1300:1800]
    # Correct output for long_text: ["A"*1500, "A"*200 + "B"*300]
    
    expected_chunk1 = "A" * 1500
    expected_chunk2 = "A" * 200 + "B" * 300
    print(f"Expected chunk 1: {expected_chunk1 == long_chunks[0]}")
    print(f"Expected chunk 2: {expected_chunk2 == long_chunks[1]}\n")

    # Test case 5: Multiple chunks with full overlap
    print("Test Case 5: Multiple chunks with full overlap")
    very_long_text = "X" * 3000
    very_long_chunks = chunker.chunk_text(very_long_text)
    print(f"Number of chunks: {len(very_long_chunks)}") # Expected: ceil((3000 - 200) / (1500 - 200)) = ceil(2800 / 1300) = ceil(2.15) = 3
    # Chunk 1: [0:1500]
    # start_index = 1300. Chunk 2: [1300:2800]
    # start_index = 1300 + 1300 = 2600. Chunk 3: [2600:3000]
    print(f"Chunk lengths: {[len(c) for c in very_long_chunks]}\n") # Expected: [1500, 1500, 400]

    # Test case 6: Custom chunk_size and overlap
    print("Test Case 6: Custom chunk_size and overlap")
    custom_chunker = TextChunker(chunk_size=10, overlap=2)
    custom_text = "0123456789abcdefghijklmnopqrstuvwxyz"
    custom_chunks = custom_chunker.chunk_text(custom_text)
    print(f"Chunks: {custom_chunks}\n")
    # Expected: ['0123456789', '89abcdefgh', 'fghijklmno', 'mnklmnopqrs', 'rstuvwxyz'] (manual check needed for this)
    # C1: 0123456789 (len 10) -> start 0
    # Next start: 0 + (10 - 2) = 8
    # C2: 89abcdefgh (len 10) -> start 8
    # Next start: 8 + 8 = 16
    # C3: ijklmnopqr (len 10) -> start 16
    # Next start: 16 + 8 = 24
    # C4: qrstuvwxyz (len 10) -> start 24
    # Chunks: ['0123456789', '89abcdefgh', 'ijklmnopqr', 'pqrstuvwxy'] -> last chunk may be shorter.
    # custom_text: 0123456789abcdefghijklmnopqrstuvwxyz (len 36)
    # 0-9 (10 chars) -> s_idx = 8
    # 8-17 (10 chars) -> s_idx = 16
    # 16-25 (10 chars) -> s_idx = 24
    # 24-33 (10 chars) -> s_idx = 32
    # 32-35 (4 chars) -> end of text
    # Actual: ['0123456789', '89abcdefgh', 'ijklmnopqr', 'pqrstuvwxy', 'yz']

    # The current implementation of `_get_module_chapter_from_path` in text_loader.py
    # assumes the module is the first subdirectory in `base_path` and chapter is the filename.
    # The `chunk_text` function doesn't use these, but it's good to keep in mind for integration.
