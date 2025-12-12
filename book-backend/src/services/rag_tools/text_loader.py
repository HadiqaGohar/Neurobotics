import os
from PyPDF2 import PdfReader

class TextLoader:
    def __init__(self, base_path: str = "modules"):
        self.base_path = base_path


    def _get_module_chapter_from_path(self, file_path: str):
        relative_path = os.path.relpath(file_path, self.base_path)
        parts = relative_path.split(os.sep)
        
        module = ""
        chapter = ""

        if len(parts) >= 2:
            module = parts[0]
            chapter = os.path.splitext(parts[-1])[0]
        elif len(parts) == 1:
            chapter = os.path.splitext(parts[0])[0]
        
        return module, chapter

    def load_text(self, file_path: str):
        module, chapter = self._get_module_chapter_from_path(file_path)
        text_content = ""
        
        file_extension = os.path.splitext(file_path)[1].lower()

        if file_extension == ".txt" or file_extension == ".md":
            with open(file_path, 'r', encoding='utf-8') as f:
                text_content = f.read()
        elif file_extension == ".pdf":
            try:
                reader = PdfReader(file_path)
                for page in reader.pages:
                    text_content += page.extract_text() + "\n"
            except Exception as e:
                print(f"Error reading PDF file {file_path}: {e}")
                text_content = "" # Or raise an error, depending on desired behavior
        else:
            print(f"Unsupported file type: {file_extension}")
            text_content = ""

        return {
            "module": module,
            "chapter": chapter,
            "text": text_content.strip()
        }

if __name__ == '__main__':
    # Example Usage (assuming a 'modules' directory in the same level as book-backend)
    # Create dummy files for testing
    os.makedirs("modules/module_test/chapter_test", exist_ok=True)
    with open("modules/module_test/chapter_test/intro.txt", "w") as f:
        f.write("This is a test chapter in text format.")
    with open("modules/module_test/chapter_test/chapter1.md", "w") as f:
        f.write("# Chapter 1\nThis is chapter one in markdown.")
    
    # You would need a dummy PDF file for full testing
    # from reportlab.pdfgen import canvas
    # c = canvas.Canvas("modules/module_test/chapter_test/sample.pdf")
    # c.drawString(100, 750, "This is a sample PDF document.")
    # c.save()

    loader = TextLoader(base_path="modules")

    txt_data = loader.load_text("modules/module_test/chapter_test/intro.txt")
    print(f"TXT Data: {txt_data}")

    md_data = loader.load_text("modules/module_test/chapter_test/chapter1.md")
    print(f"MD Data: {md_data}")

    # pdf_data = loader.load_text("modules/module_test/chapter_test/sample.pdf")
    # print(f"PDF Data: {pdf_data}")

    # Clean up dummy files
    os.remove("modules/module_test/chapter_test/intro.txt")
    os.remove("modules/module_test/chapter_test/chapter1.md")
    # os.remove("modules/module_test/chapter_test/sample.pdf")
    os.rmdir("modules/module_test/chapter_test")
    os.rmdir("modules/module_test")
    os.rmdir("modules")
