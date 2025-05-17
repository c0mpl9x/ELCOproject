import pandas as pd
import requests
import time
import json
import os
from openpyxl import load_workbook

# Configuración
API_KEY = '285c3412-feab-49ff-9bc6-b434074ecd94'  # ← tu clave aquí
API_URL = 'https://api.mouser.com/api/v1/search/partnumber'
CACHE_FILE = 'mouser_cache.json'
MAX_RETRIES = 3
RETRY_DELAY = 1  # segundos
INPUT_FILE = "componentes.xlsx"
OUTPUT_FILE = "componentes_completados_6.xlsx"

# Cargar cache si existe
if os.path.exists(CACHE_FILE):
    with open(CACHE_FILE, 'r', encoding='utf-8') as f:
        cache = json.load(f)
else:
    cache = {}

# Cargar archivo original
df_original = pd.read_excel(INPUT_FILE)
df_resultado = df_original.copy()

# Añadir columnas al resultado
df_resultado["Descripción"] = ""
df_resultado["Precio Unitario (€)"] = 0.0
df_resultado["Coste Total (€)"] = 0.0

# Función con reintentos para buscar en la API
def buscar_en_mouser(part_number):
    for intento in range(MAX_RETRIES):
        try:
            payload = {
                "SearchByPartRequest": {
                    "mouserPartNumber": part_number,
                    "partSearchOptions": "Exact"
                }
            }
            response = requests.post(
                f"{API_URL}?apiKey={API_KEY}",
                json=payload,
                headers={"Content-Type": "application/json"},
                timeout=15
            )
            if response.status_code == 200:
                return response.json()
            else:
                print(f"⚠️ [{part_number}] Status: {response.status_code}")
        except Exception as e:
            print(f"⚠️ [{part_number}] Error en intento {intento+1}: {e}")
        time.sleep(RETRY_DELAY)
    return None

# Procesar cada part number
for idx, row in df_resultado.iterrows():
    part = str(row["MouserPartNumber"]).strip()
    qty = int(row["Cantidad"])

    if part in cache:
        data = cache[part]
        print(f"🔁 Usado cache: {part}")
    else:
        response_json = buscar_en_mouser(part)
        if response_json is None:
            print(f"❌ No se pudo obtener: {part}")
            continue
        parts = response_json.get("SearchResults", {}).get("Parts", [])
        if not parts:
            print(f"❌ Vacío: {part}")
            continue
        data = parts[0]
        cache[part] = data  # Guardar en caché

    descripcion = data.get("Description", "N/A")
    price_breaks = data.get("PriceBreaks", [])
    precio = 0.0

    for b in sorted(price_breaks, key=lambda x: x["Quantity"]):
        if qty >= b["Quantity"]:
            try:
                precio = float(b["Price"].replace(",", ".").replace(" €", "").strip())
            except:
                precio = 0.0
        else:
            break

    df_resultado.at[idx, "Descripción"] = descripcion
    df_resultado.at[idx, "Precio Unitario (€)"] = precio
    df_resultado.at[idx, "Coste Total (€)"] = precio * qty

    print(f"✅ {part}: {descripcion} - {precio:.3f} €/ud")

    time.sleep(0.2)

# Añadir fila de total general
total = df_resultado["Coste Total (€)"].sum()
fila_total = {
    "MouserPartNumber": "TOTAL",
    "Cantidad": "",
    "Descripción": "",
    "Precio Unitario (€)": "",
    "Coste Total (€)": total
}
df_resultado = pd.concat([df_resultado, pd.DataFrame([fila_total])], ignore_index=True)

# Guardar en Excel con dos hojas
with pd.ExcelWriter(OUTPUT_FILE, engine="openpyxl", mode="w") as writer:
    df_original.to_excel(writer, sheet_name="Original", index=False)
    df_resultado.to_excel(writer, sheet_name="Resultados", index=False)

# Guardar caché
with open(CACHE_FILE, 'w', encoding='utf-8') as f:
    json.dump(cache, f, indent=2, ensure_ascii=False)

print(f"\n📁 Guardado: {OUTPUT_FILE}")
print(f"🗃️  Cache actualizada: {CACHE_FILE}")
print(f"💰 Coste total: {total:.2f} €")
