#include <all_function.h>

/*declare only ones
function for set your node n,nh i itd for all subs and publishers*/
int Set_publishers_subscribers(ros::NodeHandle controlnode);

/*declare only ones
funtion for connect to mavros/state and FCU connection*/
int wait_for_connect();

/*declare only ones
funtion for connect to GPS*/
int wait_for_connect_GPS_value();

/*declare only ones
funkcja dla ustalenia czasu po ktorym musi byc wysylane dane (w sekundach)
z funkcji "publish_data()"*/
int time_refresh_data(int value_period);

/*funkcaj dla wystawiania czasu dla wlasnego uzytku zwraca 1 gdy dojdzie do
wartosci jaka byla zadana*/
int timer(int value_period);
int timer2(int value_period);
int timer3(int value_period);

/*funtion for sending data what you chosed in class Myclass::value_what_want_see
wyswietlac decyduje czy chcesz zeby wartosci byly wyswietlane*/
void publish_data(bool wyswietlac);

/*wysyla  2 wiadomosci na topic "mqtt_data" liczbe i slowo
wysyla 5 razy (tak na pewniak gdy cos pojdzie cos nie tak)*/
void publish_any_value_to_topic(int number, std::string slowo);

/*funkcja do wysylania danych o dronie na topic
1-wysyla polorzenie drona po x y z 2- wysyla wartosci globalne lat lon alt
3-wysyla predkosc drona linear.x,y,y angular-x,y,z 4-wysyla aktualny czas*/
void publish_date_to_topic(bool local_values, bool globa_values, bool velocity, bool curent_time);

/*pozycja startowa(dron wzleci na odpowiednia wysykosc w tym samym polozeniu)*/
int start_position(float hight);

/*funkcja do wysylania nastepnych punktow w local wspolrzednych
1-polozenie po, x 2-po y, 3-po z, 4-obrot
tez wykonuje sie sprawdzenie czy drone dolecial do punktu dzieki funkcji
"check_waypoint_reached_local()"*/
int set_destination_local_check(float x, float y, float z, float angle);

/*funkcja do wysylania nastepnych punktow globalnych wspolrzednych
1-polozenie po latitude, 2-po longitude, 3-po altitude(ustawiasz w metrach), 4-obrot
tez wykonuje sie sprawdzenie czy drone dolecial do punktu dzieki funkcji
"check_waypoint_reached_global()"*/
int set_destination_global_check(double latitude, double longitude, double altitude, float heading);

/*funkcja do wysylania nastepnych punktow w local wspolrzednych
1-polozenie po, x 2-po y, 3-po z, 4-obrot
bez sprawdzenia na osiagniecie pozycji(waypointu)*/
void set_destination_local(float x, float y, float z, float angle);

/*funkcja do wysylania nastepnych punktow globalnych wspolrzednych
1-polozenie po latitude, 2-po longitude, 3-po altitude(ustawiasz w metrach), 4-obrot
bez sprawdzenia na osiagniecie pozycji(waypointu)*/
void set_destination_global(double latitude, double longitude, double altitude, float heading);

/*ta funkcja sprawdza czy drone dolecial do punktu z tolerancja 0.2 po x,y,z
dana funkcja sie miesci w "set_destination_local" to niema potrzeby uzywaz ich
na raz*/
int check_waypoint_reached_local();

/*ta funkcja sprawdza czy drone dolecial do punktu z tolerancja 0.2 po wartoscia globalnych
dana funkcja sie miesci w "set_destination_global" to niema potrzeby uzywaz ich
na raz*/
int check_waypoint_reached_global();

/*ta funkcja sprawdza czy drone dolecial do punktu z tolerancja 0.2 po wartoscia globalnych
ktore sa przetworzone w wartosci lokalne za pomoco funkci "convert_global_distans_to_local"
z wraca 1-dolecial 0-pozycja nie osiagnieta*/
int check_waypoint_reached_global_convert_to_local();

/*funkcja dla powrotu do punktu poczatkowego(home) oraz zmniejszenia lagodznie wysokosci
(laduje na zmnieszeniu pozycji a nie predkosci silnikow)przyjmuje wartosc
na ktorej wysokosci dron musi powrocic do bazy(local)*/
int back_home_position_local(float x, float y, float high_home);

/*funkcja dla powrotu do dowolnego punktu(home) oraz zmniejszenia lagodznie wysokosci
(laduje na zmnieszeniu pozycji a nie predkosci silnikow)przyjmuje wartosc
na ktorej wysokosci dron musi powrocic do bazy(local)*/
int back_choice_position_local(float x, float y, float high_home);

/*funkcja dla powrotu do punktu poczatkowego(home) oraz zmniejszenia lagodznie wysokosci
(laduje na zmnieszeniu pozycji a nie predkosci silnikow)przyjmuje wartosc
na ktorej wysokosci dron musi powrocic do bazy(global)*/
int back_home_position_global(float high_home);

/*funkcja dla powrotu do dowolnego punktu(home) oraz zmniejszenia lagodznie wysokosci
(laduje na zmnieszeniu pozycji a nie predkosci silnikow)przyjmuje wartosc
na ktorej wysokosci dron musi powrocic do bazy(global)*/
int back_choice_position_global(float latitude, float longitude, float high_home);

/*dron podejmuje sie dpo gory w terazniejszej pozycji dopuki nie osiagnie docelowej
wysokosci, jak osiagnie zwraca 1*/
int go_up_drone_curent_pose(float high);

/*dron zmiejsza wysokosc w terazniejszej pozycji dopuki nie osiagnie docelowej
wysokosci, jak osiagnie zwraca 1*/
int go_down_drone_curent_pose(float high);

/*funkcja do ladowania*/
int land();

/*declare only ones
funtion for reading values from file txt
ta funkcja zczytuje wartosci po linijce po kazdy elemencie
musi podac liczbe kolum ktore sa w twoim pliku oraz zmiena
see- odpowiada czy chcesz wyswietlic zapisane dane
wszystkie wartosci zapisywane do zmienych filee.x_position_file[],
filee.y_position_file[], filee.z_position_file[]*/
void read_file(std::string path, int colums, bool see);

/*funckja do konwertacji odleglosci, dajesz punkty docelowe w wartosciach GPS
po konwertacji otrzymuje wartosc odleglosci pomiedzy dronem a punktem w ukladzie
lokalnym (w x y z)*/
int convert_global_distans_to_local(double set_latitude, double set_longitude);

/*declare only ones
wysylasz true do tych funckci do ktorych chcesz podsluchiwac dane
1-global_value, 2-local_value 3-velocity value*/
void Myclass_info::value_what_want_see(bool global_info, bool local_info, bool velocity_info)
{
    open_info.global_value = global_info;
    open_info.local_value = local_info;
    open_info.value_velocity = velocity_info;
}

//////////////////////////// mission////////////////////////////

int circle(float radius, int how_many, float how_fast);
