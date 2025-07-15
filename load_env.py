Import("env")

import os

try:
    from dotenv import load_dotenv
except ImportError:
    env.Execute("$PYTHONEXE -m pip install python-dotenv")
    from dotenv import load_dotenv

load_dotenv()  # Load environment variables from .env file

# Now you can access the environment variables using os.environ
wifi_ssid = os.environ.get("HYDRO_WIFI_SSID")
wifi_password = os.environ.get("HYDRO_WIFI_PASSWORD")

# Ensure that the environment variables are set
if not wifi_ssid or not wifi_password:
    raise ValueError("HYDRO_WIFI_SSID and HYDRO_WIFI_PASSWORD must be set in the .env file")

# Example usage in your code:
print(f"Wi-Fi SSID: {wifi_ssid}")
print(f"Wi-Fi Password:", "*" * len(wifi_password))  # Print password as asterisks for security
