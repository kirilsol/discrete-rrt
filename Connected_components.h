#ifndef CONNECTED_COMPONENTS_H
#define CONNECTED_COMPONENTS_H

#include "basic_typedef.h"

typedef std::set<int>                     Connected_component;
typedef std::map<int,int>                 Int_int_map;
typedef Int_int_map::iterator             Int_int_map_iter;
typedef std::map<int,Connected_component> Int_cc_map;

class Connected_components
{
public:


public:
  //CTR & DTR
  Connected_components();
  ~Connected_components();
  //set functions
  void add_element(int elem_id);                    //add element to ds, the element is a connected component
  void connect_elements(int elem_id1,int elem_id2); //conect two connected components represented by two representing elements
  //get functions
  Connected_component& get_connected_component(int elem_id); //return all elements in element's connected component
  int                  get_connected_component_id(int elem_id);
  Int_cc_map&          get_connected_components();
  bool is_in_same_connected_component(int elem_id1,int elem_id2);
  int					get_num_connected_components();

  //dbg
  void print_connected_components();
  void print_connected_component(int cc_id);
  
private:
  int get_new_cc_id();
private:
  Int_int_map   elem_cc_map;
  Int_cc_map    id_cc_map;
  int           _id;
}; // Connected_components
#endif //CONNECTED_COMPONENTS_H