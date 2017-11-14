namespace cpp ext_iface

/******************************************************************************
 * interface objects
 ******************************************************************************/

/*! Defines what the binary in the DataRaw tranport is carrying
*/
enum TransportDef
{
	IMAGEDATA = 0, //raw data
	STRINGDATA = 1,  // name of a person
}

/*! Generic transport struct.
*/
struct TDataRaw
{
	1: required binary raw_data
}

struct TObjectResult
{
	1: required string classification
	2: required double distance
	3: required bool selected  // true if the car should drive to the person
	4: required double bbox_xmin
	5: required double bbox_ymin
	6: required double bbox_xmax
	7: required double bbox_ymax
}

struct TImageParams
{
	1: required i16 height
	2: required i16 width
	3: required i16 bytesPerPixel
	4: string name  // can be used to send a labeled image
}

typedef list<TObjectResult> TObjectResultList

/** thrown by services */
exception TIoException {
    1: string message;
}

/**
 * Generic Service for communication between two thrift entities
 * @author wormerju
 */
service ExtService {

	/*!Sends a string to a partner, receives one in return.
	 * @return a nice welcome message with the service name - where you are.
	 * @throws TIoException
	 */
	string ping(1: string sender) throws (1: TIoException ioe);

	/*!Sends raw byte data, returns a bool upon return.
	*@return true on success, false if not
	*/
  list<string> GeAvailableNames();
	TObjectResultList rawData(1: TransportDef transport_def,  2: TDataRaw raw_data, 3: TImageParams params) throws (1: TIoException ioe);
	bool IsStopped();
	void AddPerson(1: TDataRaw raw_data, 2: TImageParams params);
	void DriveToPerson(1: string name);
	void GoalReached();
  bool StartDriving();// returns true if the car starts driving 
}
