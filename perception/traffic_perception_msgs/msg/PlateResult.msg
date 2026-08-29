# A single plate-detection/OCR attempt, published once per detected plate box
# (a frame can contain more than one). Correlates the OCR'd text with its
# detection/OCR confidence and the authorization decision, which the plain
# plate_allowed (Bool) / last_plate (String) topics don't do today — those
# stay published unchanged; this is additive, for field-trial data collection.

std_msgs/Header header
string plate_text     # OCR'd plate text (may be empty if OCR found nothing)
float64 det_confidence  # plate-detector (YOLO) confidence [0,1] for this box
float64 ocr_confidence  # OCR confidence [0,1] for plate_text
bool authorized          # whether this plate matched the registered list
