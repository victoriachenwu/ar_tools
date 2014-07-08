
/* 
** Reads in transforms (in transform matrix format) from markers to center of object.
** Each marker in the marker file must have a corresponding transform.
** Meant to be used with the ar_bundle node. 
** 
** Format:
** <obj_num>	#must be same number as marker file, and same order
**
** #Transformation matrix, representing
** #transform of marker frame with respect to center frame
** 1 0 0 X
** 0 1 0 Y
** 0 0 1 Z
**
** 1 0 0 X
** 0 1 0 Y
** 0 0 1 Z
** ...
**
**	eg
**
** #transform for marker 1
** 1 0 0 20 
** 0 1 0 0 
** 0 0 1 0 
**
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <AR/ar.h>
#include <ros/ros.h>
#include <ros/package.h>

#include "ar_pose/transforms.h"

namespace ar_transforms
{

  static char *get_buff (char *buf, int n, FILE * fp);


  Transform_T *read_Transforms (char* name, int givenObjNum)	{

	Transform_T *list;
    FILE *fp;
	int objectnum;
	int i;
	int row;
	int col = 0;
    char buf[256];
      std::string package_path = ros::package::getPath (ROS_PACKAGE_NAME);

      ROS_INFO ("Opening Transform Data File %s", name);

    if ((fp = fopen (name, "r")) == NULL)
    {
      ROS_INFO ("Can't find the file - quitting");
      ROS_BREAK ();
    }

    get_buff (buf, 256, fp);
    if (sscanf (buf, "%d", &objectnum) != 1)
    {
      fclose (fp);
      ROS_BREAK ();
    }
	if(objectnum != givenObjNum)	{
	  ROS_INFO("Number of Transform matrices provided does not match number of markers provided - quitting.");
	  ROS_BREAK ();
	}

    ROS_INFO ("About to load %d Transform Matrices", givenObjNum);

	list =  (Transform_T *) malloc (sizeof (Transform_T) * givenObjNum);
    if (list == NULL)
      ROS_BREAK ();


	printf("--------------------------\n");
	for (i = 0; i< givenObjNum; i++)	{
	  printf("reading in matrix for marker #%d\n", i);
      for( row = 0; row <  3; row++)	{
			  get_buff (buf, 256, fp);
			  printf("%s", buf);
			  if (sscanf (buf, "%lf %lf %lf %lf", 
						  &list[i][row][col], &list[i][row][col+1], &list[i][row][col+2], &list[i][row][col+3]) != 4)
			  {
				fclose (fp);
				free (list);
				ROS_BREAK ();
			  }
	  }

	}
	ROS_INFO("Finished loading matrices.");


	//let's print them.. see if it worked O.O
	printf("--------------------------\n");
	for(i = 0; i< givenObjNum; i++)	{
		printf("stored matrix for marker #%d\n", i);
		for(row = 0; row < 3; row++)	{
			for (col = 0; col< 4 ; col++) printf("%f ", list[i][row][col]);	
			printf("\n");
		}
		printf("\n");
	}
	return list;

  }
  /*
  ObjectData_T *read_ObjData (char *name, int *objectnum)
  {
    FILE *fp;
    ObjectData_T *object;
    char buf[256], buf1[256];
    int i;
      std::string package_path = ros::package::getPath (ROS_PACKAGE_NAME);

      ROS_INFO ("Opening Data File %s", name);

    if ((fp = fopen (name, "r")) == NULL)
    {
      ROS_INFO ("Can't find the file - quitting");
      ROS_BREAK ();
    }

    get_buff (buf, 256, fp);
    if (sscanf (buf, "%d", objectnum) != 1)
    {
      fclose (fp);
      ROS_BREAK ();
    }

    ROS_INFO ("About to load %d Models", *objectnum);

    object = (ObjectData_T *) malloc (sizeof (ObjectData_T) * *objectnum);
    if (object == NULL)
      ROS_BREAK ();

    for (i = 0; i < *objectnum; i++)
    {
      object[i].visible = 0;

      get_buff (buf, 256, fp);
      if (sscanf (buf, "%s", object[i].name) != 1)
      {
        fclose (fp);
        free (object);
        ROS_BREAK ();
      }

      ROS_INFO ("Read in No.%d", i + 1);

      get_buff (buf, 256, fp);
      if (sscanf (buf, "%s", buf1) != 1)
      {
        fclose (fp);
        free (object);
        ROS_BREAK ();
      }

      sprintf (buf, "%s/%s", package_path.c_str (), buf1);
      if ((object[i].id = arLoadPatt (buf)) < 0)
      {
        fclose (fp);
        free (object);
        ROS_BREAK ();
      }

      get_buff (buf, 256, fp);
      if (sscanf (buf, "%lf", &object[i].marker_width) != 1)
      {
        fclose (fp);
        free (object);
        ROS_BREAK ();
      }

      get_buff (buf, 256, fp);
      if (sscanf (buf, "%lf %lf", &object[i].marker_center[0], &object[i].marker_center[1]) != 2)
      {
        fclose (fp);
        free (object);
        ROS_BREAK ();
      }
    }

    fclose (fp);

    return (object);
  }

*/
  //get up to n chars, disregarding blank lines and lines starting with #s
  static char *get_buff (char *buf, int n, FILE * fp)
  {
    char *ret;

    for (;;)
    {
      ret = fgets (buf, n, fp);
      if (ret == NULL)
        return (NULL);
      if (buf[0] != '\n' && buf[0] != '#')
        return (ret);
    }
  }
}
