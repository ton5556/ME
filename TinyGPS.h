// ... (previous includes and declarations)

TinyGPS gps;

GeoLoc checkGPS() {
  Serial.println("Reading onboard GPS: ");
  bool newData = false;
  unsigned long start = millis();
  
  // For one second we parse GPS data and report some key values
  while (millis() - start < GPS_UPDATE_INTERVAL) {
    while (GPSSerial.available()) {
      char c = GPSSerial.read();
      // Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }

  GeoLoc coolerLoc;
  
  if (newData) {
    float flat, flon;
    unsigned long age;
    gps.f_get_position(&flat, &flon, &age);
    coolerLoc.lat = flat;
    coolerLoc.lon = flon;
    
    Serial.print("LAT=");
    Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    Serial.print(" LON=");
    Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
    Serial.print(" SAT=");
    Serial.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
    Serial.print(" PREC=");
    Serial.print(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());
  } else {
    coolerLoc.lat = 0.0;
    coolerLoc.lon = 0.0;
    Serial.println("No valid GPS data received");
  }
  
  return coolerLoc;
}

// ... (rest of the code)
