import React, { useState, useRef, useEffect } from "react";
import { VoiceInputProps } from "./types";

const VoiceInput: React.FC<VoiceInputProps> = ({
  onVoiceInput,
  isRecording,
  onToggleRecording,
}) => {
  const [mediaRecorder, setMediaRecorder] = useState<MediaRecorder | null>(
    null
  );
  const [audioChunks, setAudioChunks] = useState<Blob[]>([]);
  const [isSupported, setIsSupported] = useState(true);

  useEffect(() => {
    // Check if browser supports media recording
    if (typeof window === 'undefined' || !navigator.mediaDevices || !window.MediaRecorder) {
      setIsSupported(false);
    }
  }, []);

  const startRecording = async () => {
    if (typeof window === 'undefined') return;
    
    try {
      const stream = await navigator.mediaDevices.getUserMedia({ audio: true });
      const recorder = new MediaRecorder(stream);

      recorder.ondataavailable = (event) => {
        if (event.data.size > 0) {
          setAudioChunks((prev) => [...prev, event.data]);
        }
      };

      recorder.onstop = () => {
        const audioBlob = new Blob(audioChunks, { type: "audio/wav" });
        onVoiceInput(audioBlob);
        setAudioChunks([]);

        // Stop all tracks to release microphone
        stream.getTracks().forEach((track) => track.stop());
      };

      recorder.start();
      setMediaRecorder(recorder);
    } catch (error) {
      console.error("Error accessing microphone:", error);
      alert("Unable to access microphone. Please check your permissions.");
    }
  };

  const stopRecording = () => {
    if (mediaRecorder && mediaRecorder.state === "recording") {
      mediaRecorder.stop();
      setMediaRecorder(null);
    }
  };

  const handleToggleRecording = () => {
    if (isRecording) {
      stopRecording();
    } else {
      startRecording();
    }
    onToggleRecording();
  };

  if (!isSupported) {
    return null; // Hide voice input if not supported
  }

  return (
    <button
      className={`control-btn voice-btn ${isRecording ? "recording" : ""}`}
      onClick={handleToggleRecording}
      aria-label={isRecording ? "Stop recording" : "Start voice input"}
      title={isRecording ? "Stop recording" : "Voice input"}
    >
      {isRecording ? (
        <svg width="16" height="16" viewBox="0 0 24 24" fill="currentColor">
          <rect x="6" y="6" width="12" height="12" rx="2"></rect>
        </svg>
      ) : (
        <svg
          width="16"
          height="16"
          viewBox="0 0 24 24"
          fill="none"
          stroke="currentColor"
          strokeWidth="2"
        >
          <path d="M12 1a3 3 0 0 0-3 3v8a3 3 0 0 0 6 0V4a3 3 0 0 0-3-3z"></path>
          <path d="M19 10v2a7 7 0 0 1-14 0v-2"></path>
          <line x1="12" y1="19" x2="12" y2="23"></line>
          <line x1="8" y1="23" x2="16" y2="23"></line>
        </svg>
      )}

      {isRecording && (
        <div className="recording-pulse">
          <div className="pulse-ring"></div>
        </div>
      )}
    </button>
  );
};

export default VoiceInput;
