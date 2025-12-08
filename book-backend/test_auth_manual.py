#!/usr/bin/env python3
"""
Manual test script for authentication endpoints
Run this to test the authentication system manually
"""

import requests
import json
import sys

BASE_URL = "http://localhost:8000/api/v1/auth"

def test_registration():
    """Test user registration"""
    print("Testing user registration...")
    
    user_data = {
        "email": "test@example.com",
        "password": "TestPass123!",
        "confirm_password": "TestPass123!",
        "full_name": "Test User"
    }
    
    try:
        response = requests.post(f"{BASE_URL}/register", json=user_data)
        print(f"Registration Status: {response.status_code}")
        
        if response.status_code == 201:
            data = response.json()
            print("✅ Registration successful!")
            print(f"User ID: {data['user']['id']}")
            print(f"Email: {data['user']['email']}")
            return data['access_token'], data['user']['email']
        else:
            print(f"❌ Registration failed: {response.text}")
            return None, None
            
    except requests.exceptions.ConnectionError:
        print("❌ Could not connect to server. Make sure the backend is running on localhost:8000")
        return None, None
    except Exception as e:
        print(f"❌ Error during registration: {e}")
        return None, None

def test_login(email, password="TestPass123!"):
    """Test user login"""
    print(f"\nTesting login for {email}...")
    
    login_data = {
        "email": email,
        "password": password
    }
    
    try:
        response = requests.post(f"{BASE_URL}/login", json=login_data)
        print(f"Login Status: {response.status_code}")
        
        if response.status_code == 200:
            data = response.json()
            print("✅ Login successful!")
            return data['access_token']
        else:
            print(f"❌ Login failed: {response.text}")
            return None
            
    except Exception as e:
        print(f"❌ Error during login: {e}")
        return None

def test_protected_endpoint(token):
    """Test accessing protected endpoint"""
    print(f"\nTesting protected endpoint...")
    
    headers = {
        "Authorization": f"Bearer {token}"
    }
    
    try:
        response = requests.get(f"{BASE_URL}/me", headers=headers)
        print(f"Protected endpoint Status: {response.status_code}")
        
        if response.status_code == 200:
            data = response.json()
            print("✅ Protected endpoint access successful!")
            print(f"User: {data['email']}")
            return True
        else:
            print(f"❌ Protected endpoint access failed: {response.text}")
            return False
            
    except Exception as e:
        print(f"❌ Error accessing protected endpoint: {e}")
        return False

def test_rate_limiting():
    """Test rate limiting"""
    print(f"\nTesting rate limiting...")
    
    # Try to register multiple times quickly
    user_data = {
        "email": "spam@example.com",
        "password": "TestPass123!",
        "confirm_password": "TestPass123!",
        "full_name": "Spam User"
    }
    
    for i in range(7):  # Try 7 times (limit is 5)
        try:
            response = requests.post(f"{BASE_URL}/register", json=user_data)
            print(f"Attempt {i+1}: Status {response.status_code}")
            
            if response.status_code == 429:
                print("✅ Rate limiting is working!")
                return True
                
        except Exception as e:
            print(f"Error in attempt {i+1}: {e}")
    
    print("⚠️ Rate limiting test inconclusive")
    return False

def main():
    """Run all tests"""
    print("🚀 Starting Authentication System Tests")
    print("=" * 50)
    
    # Test registration
    token, email = test_registration()
    
    if not token:
        print("\n❌ Registration failed, skipping other tests")
        return
    
    # Test login
    login_token = test_login(email)
    
    if not login_token:
        print("\n❌ Login failed, using registration token for protected endpoint test")
        login_token = token
    
    # Test protected endpoint
    test_protected_endpoint(login_token)
    
    # Test rate limiting
    test_rate_limiting()
    
    print("\n" + "=" * 50)
    print("🏁 Authentication tests completed!")
    print("\nTo test the frontend:")
    print("1. Start the frontend: cd book-frontend && npm start")
    print("2. Visit http://localhost:3000/auth/signup")
    print("3. Try registering and logging in")

if __name__ == "__main__":
    main()