# frAIburg Dynamic CSV Writer

**author: Markus Merklinger**
**abstract:**
- filter to write data to a csv file
- sampling time and output file name can be set in the filter properties,
- the file is stored under recordings/filename.csv, the filename is changed
  if it exists, ensure that the recordings folder exist or creation is possible
- the first column is a timestamp
- the column title is the dynamic pin name
**inputs:**
- dynamic inputs with supported types: tSignalValue, tPosition, tWheelData,
  tInerMeasUnitData
