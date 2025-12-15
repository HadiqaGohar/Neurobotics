from typing import Optional, List
from datetime import datetime
from pydantic import BaseModel, Field

# Pydantic Models for Books

class BookBase(BaseModel):
    title: str = Field(..., max_length=255)
    description: Optional[str] = Field(None, max_length=5000)
    isbn: Optional[str] = Field(None, max_length=17, regex=r"^(?:ISBN(?:-13)?:?)(?=[0-9]{13}$)([0-9]{3}-){2}[0-9]{3}[0-9X]$")
    language: Optional[str] = Field("en", max_length=10)
    category: Optional[str] = Field(None, max_length=100)
    tags: Optional[List[str]] = Field(None)
    is_published: bool = False
    is_featured: bool = False

class BookCreate(BookBase):
    pass

class BookUpdate(BookBase):
    title: Optional[str] = Field(None, max_length=255)
    description: Optional[str] = Field(None, max_length=5000)
    isbn: Optional[str] = Field(None, max_length=17, regex=r"^(?:ISBN(?:-13)?:?)(?=[0-9]{13}$)([0-9]{3}-){2}[0-9]{3}[0-9X]$")

class BookInDB(BookBase):
    id: int
    author_id: int
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True

# Pydantic Models for Chapters

class ChapterBase(BaseModel):
    title: str = Field(..., max_length=255)
    content: str
    chapter_number: int = Field(..., ge=1)
    word_count: Optional[int] = Field(None, ge=0)
    reading_time: Optional[int] = Field(None, ge=0)

class ChapterCreate(ChapterBase):
    pass

class ChapterUpdate(ChapterBase):
    title: Optional[str] = Field(None, max_length=255)
    content: Optional[str] = None
    chapter_number: Optional[int] = Field(None, ge=1)

class ChapterInDB(ChapterBase):
    id: int
    book_id: int
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True
