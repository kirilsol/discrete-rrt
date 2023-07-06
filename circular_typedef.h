#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Gps_circle_segment_traits_2.h>
#include <CGAL/General_polygon_set_2.h>
#include "basic_typedef.h"
#include <CGAL/Arr_trapezoid_ric_point_location.h>
#include <CGAL/Arr_landmarks_point_location.h>
#include <CGAL/Arr_walk_along_line_point_location.h>

typedef CGAL::Gps_circle_segment_traits_2<Kernel>			Gps_traits_2;
typedef Gps_traits_2::Polygon_2								Offset_polygon_2;
typedef Gps_traits_2::Polygon_with_holes_2					Offset_polygon_with_holes_2;

typedef	CGAL::General_polygon_set_2<Gps_traits_2>			Offset_polygon_set_2;
typedef	Offset_polygon_set_2::Arrangement_2					Offset_arrangement;
typedef	Offset_arrangement::Point_2							Offset_point;

//typedef	CGAL::Arr_trapezoid_ric_point_location<Offset_arrangement>	Offset_ric_pl;
typedef	CGAL::Arr_walk_along_line_point_location<Arrangement_2>		Ric_pl;

//typedef	CGAL::Arr_landmarks_point_location<Offset_arrangement>	Offset_ric_pl;
//typedef	CGAL::Arr_walk_along_line_point_location<Arrangement_2>		Ric_pl;

