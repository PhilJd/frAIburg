

/******************************************************************************
 * interface objects for the frAIburg webapp
 ******************************************************************************/

struct TAppDataRaw
{
	1: required binary raw_data
}


struct TImageParams
{
	1: required i16 height
	2: required i16 width
	3: string name  // can be used to send a labeled image
}


service NewService {
  string ping(1:string s1);
  list<string> get_all_names(); //returs all aviable names to select
  bool add_person(1: TAppDataRaw raw_data, 2: TImageParams params); // return ture only if a new person was added
  bool remove_person(1:string s);//return ture if person was removed
  bool drive_to(1:string s1); //return false if error occured
  bool start_driving();
}
