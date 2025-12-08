from sqlalchemy.orm import Session
from typing import List, Optional
from src.database.models import DocumentChunk, Session as DBSession, Message, Document
import uuid

def get_document_chunk_content_by_embedding_ids(db: Session, embedding_ids: List[str]) -> List[str]:
    chunks = db.query(DocumentChunk).filter(DocumentChunk.embedding_id.in_(embedding_ids)).all()
    # Preserve the order of the retrieved chunks to match the order of embedding_ids
    chunk_map = {chunk.embedding_id: chunk.content for chunk in chunks}
    return [chunk_map[e_id] for e_id in embedding_ids if e_id in chunk_map]

def create_session(db: Session, user_id: Optional[uuid.UUID] = None) -> DBSession:
    db_session = DBSession(user_id=user_id)
    db.add(db_session)
    db.commit()
    db.refresh(db_session)
    return db_session

def get_session(db: Session, session_id: uuid.UUID) -> Optional[DBSession]:
    return db.query(DBSession).filter(DBSession.id == session_id).first()

def create_message(db: Session, session_id: uuid.UUID, content: str, sender: str, extra_data: Optional[dict] = None) -> Message:
    db_message = Message(session_id=session_id, content=content, sender=sender, extra_data=extra_data)
    db.add(db_message)
    db.commit()
    db.refresh(db_message)
    return db_message

def get_messages_by_session_id(db: Session, session_id: uuid.UUID) -> List[Message]:
    return db.query(Message).filter(Message.session_id == session_id).order_by(Message.timestamp).all()



def create_document(db: Session, title: str, source_uri: str, total_chunks: Optional[int] = None) -> Document:
    db_document = Document(title=title, source_uri=source_uri, total_chunks=total_chunks)
    db.add(db_document)
    db.commit()
    db.refresh(db_document)
    return db_document

def create_document_chunk(db: Session, document_id: uuid.UUID, chunk_index: int, content: str, embedding_id: str, extra_data: Optional[dict] = None) -> DocumentChunk:
    db_chunk = DocumentChunk(document_id=document_id, chunk_index=chunk_index, content=content, embedding_id=embedding_id, extra_data=extra_data)
    db.add(db_chunk)
    db.commit()
    db.refresh(db_chunk)
    return db_chunk
