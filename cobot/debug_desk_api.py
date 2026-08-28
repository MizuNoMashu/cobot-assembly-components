#!/usr/bin/env python3
"""
Script di debug per testare la FrankaDeskAPI
Usa SEMPRE HTTPS perché il robot redirige HTTP→HTTPS automaticamente
"""
import sys
sys.path.insert(0, './src')
import os
from franka_controller.desk_api import FrankaDeskAPI

print("=" * 70)
print("TEST: FrankaDeskAPI - HTTPS + Credenziali Corrette")
print("=" * 70)

try:
    print("\n[1/4] Creazione client FrankaDeskAPI...")
    desk = FrankaDeskAPI(
        robot_ip=os.environ.get("robot_ip", ""),
        scheme="https",  # ← IMPORTANTE: Sempre HTTPS
        username=os.environ.get("username", ""),
        password=os.environ.get("password", ""),
    )
    print("  ✓ Client creato")
    
    print("\n[2/4] Test GET /api/system...")
    system = desk.get_system_state()
    print(f"  ✓ Stato sistema: {system.get('status')}")
    
    print("\n[3/4] Test GET /api/system/control-token (stato corrente)...")
    token_state = desk.get_control_token_state()
    print(f"  ✓ Token state: {token_state}")
    
    print("\n[4/4] Test POST /api/system/control-token:take (ACQUISIRE TOKEN)...")
    token = desk.take_control_token(timeout=10.0)
    print(f"  ✓✓✓ TOKEN ACQUISITO CON SUCCESSO!")
    print(f"      Token ID: {token.token_id}")
    print(f"      Owner: {token.owner}")
    print(f"      Token: {token.token[:50]}...")
    
    print("\n" + "=" * 70)
    print("✓✓✓ TUTTO OK! La FrankaDeskAPI funziona correttamente")
    print("=" * 70)
    


    
except Exception as e:
    print(f"\n✗✗✗ ERRORE:")
    print(f"  Tipo: {type(e).__name__}")
    print(f"  Messaggio: {e}")
    print("\n" + "=" * 70)
