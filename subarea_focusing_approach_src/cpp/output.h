/* Portions Copyright 2019-2021 Xuesong Zhou and Peiheng Li, Cafer Avci

 * If you help write or modify the code, please also list your names here.
 * The reason of having Copyright info here is to ensure all the modified version, as a whole, under the GPL
 * and further prevent a violation of the GPL.
 *
 * More about "How to use GNU licenses for your own software"
 * http://www.gnu.org/licenses/gpl-howto.html
 */

 // Peiheng, 02/03/21, remove them later after adopting better casting
#pragma warning(disable : 4305 4267 4018)
// stop warning: "conversion from 'int' to 'float', possible loss of data"
#pragma warning(disable: 4244)

#ifdef _WIN32
#include "pch.h"
#endif

#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <string>
#include <cstring>
#include <cstdio>
#include <ctime>
#include <cmath>
#include <algorithm>
#include <functional>
#include <stack>
#include <list>
#include <vector>
#include <map>
#include <omp.h>
#include "config.h"
#include "utils.h"


using std::max;
using std::min;
using std::cout;
using std::endl;
using std::string;
using std::vector;
using std::map;
using std::ifstream;
using std::ofstream;
using std::istringstream;

#include "DTA.h"

void g_OutputModelFiles(int mode)
{
	{
		int connector_count = 0;
		for (int i = 0; i < g_link_vector.size(); i++)
		{
			if (g_link_vector[i].link_type == 1000)  // connector
			{
				connector_count += 1;
			}
		}

		if (connector_count >= 1)
		{
			FILE* g_pFileModelLink = fopen("access_link.csv", "w");

			if (g_pFileModelLink != NULL)
			{
				fprintf(g_pFileModelLink, "link_id,link_no,from_node_id,to_node_id,link_type,link_type_name,lanes,link_distance_VDF,free_speed,fftt,capacity,allow_uses,geometry\n");

				//VDF_fftt1,VDF_cap1,VDF_alpha1,VDF_beta1
				for (int i = 0; i < g_link_vector.size(); i++)
				{
					if (g_link_vector[i].link_type == 1000)  // connector
					{

						fprintf(g_pFileModelLink, "%s,%d,%d,%d,%d,%s,%d,%f,%f,%f,%f,%s,",
							g_link_vector[i].link_id.c_str(),
							g_link_vector[i].link_seq_no,
							g_node_vector[g_link_vector[i].from_node_seq_no].node_id,
							g_node_vector[g_link_vector[i].to_node_seq_no].node_id,
							g_link_vector[i].link_type,
							g_link_vector[i].link_type_name.c_str(),
							g_link_vector[i].number_of_lanes,
							g_link_vector[i].link_distance_VDF,
							g_link_vector[i].free_speed,
							g_link_vector[i].free_flow_travel_time_in_min,
							g_link_vector[i].lane_capacity,
//							g_link_vector[i].VDF_period[0].allowed_uses.c_str()
							//g_link_vector[i].VDF_period[0].FFTT,
						   //g_link_vector[i].VDF_period[0].period_capacity,
						   //g_link_vector[i].VDF_period[0].alpha,
						   //g_link_vector[i].VDF_period[0].beta,
						);

						if (g_link_vector[i].geometry.size() > 0)
						{
							fprintf(g_pFileModelLink, "\"%s\",", g_link_vector[i].geometry.c_str());
						}
						else
						{
							fprintf(g_pFileModelLink, "\"LINESTRING (");

							fprintf(g_pFileModelLink, "%f %f,", g_node_vector[g_link_vector[i].from_node_seq_no].x, g_node_vector[g_link_vector[i].from_node_seq_no].y);
							fprintf(g_pFileModelLink, "%f %f", g_node_vector[g_link_vector[i].to_node_seq_no].x, g_node_vector[g_link_vector[i].to_node_seq_no].y);

							fprintf(g_pFileModelLink, ")\"");
						}

						fprintf(g_pFileModelLink, "\n");

					}


				}

				fclose(g_pFileModelLink);
			}
			else
			{
				dtalog.output() << "Error: File access_link.csv cannot be opened.\n It might be currently used and locked by EXCEL." << endl;
				g_program_stop();

			}

		}

	}

	if (mode == 3)  // cell
	{
		//FILE* g_pFileZone = nullptr;
		//g_pFileZone = fopen("model_cell.csv", "w");

		//if (g_pFileZone == NULL)
		//{
		//    cout << "File model_cell.csv cannot be opened." << endl;
		//    g_program_stop();
		//}
		//else
		//{


		//    fprintf(g_pFileZone, "cell_code,geometry\n");

		//    std::map<string, CInfoCell>::iterator it;

		//    for (it = g_info_cell_map.begin(); it != g_info_cell_map.end(); ++it)
		//    {

		//        fprintf(g_pFileZone, "%s,", it->first.c_str());
		//        fprintf(g_pFileZone, "\"LINESTRING (");

		//        for (int s = 0; s < it->second.m_ShapePoints.size(); s++)
		//        {
		//            fprintf(g_pFileZone, "%f %f,", it->second.m_ShapePoints[s].x, it->second.m_ShapePoints[s].y);
		//        }

		//        fprintf(g_pFileZone, ")\"");
		//        fprintf(g_pFileZone, "\n");
		//    }
		//    fclose(g_pFileZone);
		//}
	}

	if (mode == 10)
	{
		FILE* g_pFileModel_LC = fopen("model_shortest_path_tree.csv", "w");

		if (g_pFileModel_LC != NULL)
		{
			fprintf(g_pFileModel_LC, "iteration,agent_type,zone_id,node_id,d_zone_id,connected_flag,pred,label_cost,pred_link_cost,x_coord,y_coord,geometry\n");
			for (int i = 0; i < g_node_vector.size(); i++)
			{


				//                if (g_node_vector[i].node_id >= 0)  //for all physical links
				{
					std::map<string, float> ::iterator it;

					for (it = g_node_vector[i].label_cost_per_iteration_map.begin(); it != g_node_vector[i].label_cost_per_iteration_map.end(); ++it)
					{
						int node_pred_id = -1;
						int pred_no = g_node_vector[i].pred_per_iteration_map[it->first];
						float pred_link_cost = 0;
						if (pred_no >= 0)
						{
							std::map<string, float> ::iterator it2;

							float pred_node_label_cost = 0;
							for (it2 = g_node_vector[pred_no].label_cost_per_iteration_map.begin(); it2 != g_node_vector[pred_no].label_cost_per_iteration_map.end(); ++it2)
							{
								pred_node_label_cost = it2->second;
							}

							pred_link_cost = it->second - pred_node_label_cost;
							node_pred_id = g_node_vector[pred_no].node_id;

						}
						int d_zone_id = g_node_vector[i].zone_id;
						int connected_flag = 0;

						if (it->second < 100000)
							connected_flag = 1;

						if (connected_flag == 1)
						{
							fprintf(g_pFileModel_LC, "%s,%d,%d,%d,%f,%f,%f,%f,", it->first.c_str(), d_zone_id, connected_flag, node_pred_id, it->second,
								pred_link_cost,
								g_node_vector[i].x, g_node_vector[i].y);

							int pred_no_2 = i;
							if (pred_no >= 0)
								pred_no_2 = pred_no;

							fprintf(g_pFileModel_LC, "\"LINESTRING (");
							fprintf(g_pFileModel_LC, "%f %f,", g_node_vector[i].x, g_node_vector[i].y);

							fprintf(g_pFileModel_LC, "%f %f", g_node_vector[pred_no_2].x, g_node_vector[pred_no_2].y);

							fprintf(g_pFileModel_LC, ")\"");

							fprintf(g_pFileModel_LC, "\n");
						}
					}

				}

			}

			fclose(g_pFileModel_LC);
		}
		else
		{
			dtalog.output() << "Error: File model_label_cost_tree.csv cannot be opened.\n It might be currently used and locked by EXCEL." << endl;
			g_program_stop();


		}

	}

	if(mode  == 20)
	{
	FILE* g_pFileModel_LC = fopen("model_RT_shortest_tree.csv", "w");

	if (g_pFileModel_LC != NULL)
	{
		fprintf(g_pFileModel_LC, "demand_period,agent_type,d_zone_id,node_id,o_zone_id,connected_flag,pred,label_cost,geometry\n");
		for (int i = 0; i < g_node_vector.size(); i++)
		{
			//                if (g_node_vector[i].node_id >= 0)  //for all physical links
			{
				std::map<string, float> ::iterator it;

				for (it = g_node_vector[i].label_cost_RT_map.begin(); it != g_node_vector[i].label_cost_RT_map.end(); ++it)
				{
					int node_pred_id = -1;
					int pred_no = g_node_vector[i].pred_RT_map[it->first];
					if (pred_no >= 0)
						node_pred_id = g_node_vector[pred_no].node_id;
					int o_zone_id = g_node_vector[i].zone_id;
					int connected_flag = 0;

					if (it->second < 1000)  // feasible link only
					{
						connected_flag = 1;
						fprintf(g_pFileModel_LC, "%s,%d,%d,%d,%f,", it->first.c_str(), o_zone_id, connected_flag, node_pred_id, it->second);

						if (node_pred_id >= 0)
						{
							fprintf(g_pFileModel_LC, "\"LINESTRING (");
							fprintf(g_pFileModel_LC, "%f %f,", g_node_vector[i].x, g_node_vector[i].y);
							fprintf(g_pFileModel_LC, "%f %f,", g_node_vector[pred_no].x, g_node_vector[pred_no].y);
							fprintf(g_pFileModel_LC, ")\"");
							fprintf(g_pFileModel_LC, ",");
						}

						fprintf(g_pFileModel_LC, "\n");
					}
				}

			}

		}

		fclose(g_pFileModel_LC);
	}
	}
}
