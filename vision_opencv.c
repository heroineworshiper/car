#include <stdio.h>
#include <stdlib.h>

#include "vision.h"


#ifdef USE_OPENCV

#include <iostream>
#include <vector>
#include "opencv2/legacy/compat.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv2/imgproc/types_c.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/video/background_segm.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"


// detect path by matching images with OpenCV
using namespace cv;
using namespace std;

IplImage *object_image = 0;
IplImage *scene_image = 0;
CvMemStorage *storage = 0;

CvSeq *object_keypoints = 0;
CvSeq *object_descriptors = 0;
CvSeq *scene_keypoints = 0;
CvSeq *scene_descriptors = 0;
int opencv_initialized = 0;

// reference file
#define REF_PATH "ref.in"
#define REF_FRAMES 100
unsigned char *ref_y[REF_FRAMES];
unsigned char *ref_u[REF_FRAMES];
unsigned char *ref_v[REF_FRAMES];
FILE *ref_fd;


// define whether to use approximate nearest-neighbor search
#define USE_FLANN


double
compareSURFDescriptors( const float* d1, const float* d2, double best, int length )
{
    double total_cost = 0;
    assert( length % 4 == 0 );
    for( int i = 0; i < length; i += 4 )
    {
        double t0 = d1[i  ] - d2[i  ];
        double t1 = d1[i+1] - d2[i+1];
        double t2 = d1[i+2] - d2[i+2];
        double t3 = d1[i+3] - d2[i+3];
        total_cost += t0*t0 + t1*t1 + t2*t2 + t3*t3;
        if( total_cost > best )
            break;
    }
    return total_cost;
}


int
naiveNearestNeighbor( const float* vec, int laplacian,
                      const CvSeq* model_keypoints,
                      const CvSeq* model_descriptors )
{
    int length = (int)(model_descriptors->elem_size/sizeof(float));
    int i, neighbor = -1;
    double d, dist1 = 1e6, dist2 = 1e6;
    CvSeqReader reader, kreader;
    cvStartReadSeq( model_keypoints, &kreader, 0 );
    cvStartReadSeq( model_descriptors, &reader, 0 );

    for( i = 0; i < model_descriptors->total; i++ )
    {
        const CvSURFPoint* kp = (const CvSURFPoint*)kreader.ptr;
        const float* mvec = (const float*)reader.ptr;
    	CV_NEXT_SEQ_ELEM( kreader.seq->elem_size, kreader );
        CV_NEXT_SEQ_ELEM( reader.seq->elem_size, reader );
        if( laplacian != kp->laplacian )
            continue;
        d = compareSURFDescriptors( vec, mvec, dist2, length );
        if( d < dist1 )
        {
            dist2 = dist1;
            dist1 = d;
            neighbor = i;
        }
        else if ( d < dist2 )
            dist2 = d;
    }
    if ( dist1 < 0.6*dist2 )
        return neighbor;
    return -1;
}

void
findPairs( const CvSeq* objectKeypoints, const CvSeq* objectDescriptors,
           const CvSeq* imageKeypoints, const CvSeq* imageDescriptors, vector<int>& ptpairs )
{
    int i;
    CvSeqReader reader, kreader;
    cvStartReadSeq( objectKeypoints, &kreader );
    cvStartReadSeq( objectDescriptors, &reader );
    ptpairs.clear();

    for( i = 0; i < objectDescriptors->total; i++ )
    {
        const CvSURFPoint* kp = (const CvSURFPoint*)kreader.ptr;
        const float* descriptor = (const float*)reader.ptr;
        CV_NEXT_SEQ_ELEM( kreader.seq->elem_size, kreader );
        CV_NEXT_SEQ_ELEM( reader.seq->elem_size, reader );
        int nearest_neighbor = naiveNearestNeighbor( descriptor, kp->laplacian, imageKeypoints, imageDescriptors );
        if( nearest_neighbor >= 0 )
        {
            ptpairs.push_back(i);
            ptpairs.push_back(nearest_neighbor);
        }
    }
}


void
flannFindPairs( const CvSeq*, 
	const CvSeq* objectDescriptors,
    const CvSeq*, 
	const CvSeq* imageDescriptors, 
	vector<int>& ptpairs )
{
	int length = (int)(objectDescriptors->elem_size/sizeof(float));

    cv::Mat m_object(objectDescriptors->total, length, CV_32F);
	cv::Mat m_image(imageDescriptors->total, length, CV_32F);


	// copy descriptors
    CvSeqReader obj_reader;
	float* obj_ptr = m_object.ptr<float>(0);
    cvStartReadSeq( objectDescriptors, &obj_reader );
    for(int i = 0; i < objectDescriptors->total; i++ )
    {
        const float* descriptor = (const float*)obj_reader.ptr;
        CV_NEXT_SEQ_ELEM( obj_reader.seq->elem_size, obj_reader );
        memcpy(obj_ptr, descriptor, length*sizeof(float));
        obj_ptr += length;
    }
    CvSeqReader img_reader;
	float* img_ptr = m_image.ptr<float>(0);
    cvStartReadSeq( imageDescriptors, &img_reader );
    for(int i = 0; i < imageDescriptors->total; i++ )
    {
        const float* descriptor = (const float*)img_reader.ptr;
        CV_NEXT_SEQ_ELEM( img_reader.seq->elem_size, img_reader );
        memcpy(img_ptr, descriptor, length*sizeof(float));
        img_ptr += length;
    }

    // find nearest neighbors using FLANN
    cv::Mat m_indices(objectDescriptors->total, 2, CV_32S);
    cv::Mat m_dists(objectDescriptors->total, 2, CV_32F);
    cv::flann::Index flann_index(m_image, cv::flann::KDTreeIndexParams(4));  // using 4 randomized kdtrees
    flann_index.knnSearch(m_object, m_indices, m_dists, 2, cv::flann::SearchParams(64) ); // maximum number of leafs checked

    int* indices_ptr = m_indices.ptr<int>(0);
    float* dists_ptr = m_dists.ptr<float>(0);
//printf("flannFindPairs %d m_indices.rows=%d\n", __LINE__, m_indices.rows);
    for (int i = 0; i < m_indices.rows; ++i) 
	{
//printf("flannFindPairs %d dists=%f %f\n", __LINE__, dists_ptr[2 * i], 0.6 * dists_ptr[2 * i + 1]);
    	if (dists_ptr[2 * i] < 0.6 * dists_ptr[2 * i + 1]) 
		{
//printf("flannFindPairs %d pairs=%d\n", __LINE__, ptpairs.size());
    		ptpairs.push_back(i);
    		ptpairs.push_back(indices_ptr[2*i]);
    	}
    }
}


/* a rough implementation for object location */
int
locatePlanarObject(const CvSeq* objectKeypoints, 
	const CvSeq* objectDescriptors,
    const CvSeq* imageKeypoints, 
	const CvSeq* imageDescriptors,
    const CvPoint src_corners[4], 
	CvPoint dst_corners[4],
	int *(*point_pairs),
	int (*total_pairs))
{
    double h[9];
    CvMat _h = cvMat(3, 3, CV_64F, h);
    vector<int> ptpairs;
    vector<CvPoint2D32f> pt1, pt2;
    CvMat _pt1, _pt2;
    int i, n;
	
	(*point_pairs) = 0;
	(*total_pairs) = 0;

#ifdef USE_FLANN
    flannFindPairs( objectKeypoints, objectDescriptors, imageKeypoints, imageDescriptors, ptpairs );
#else
    findPairs( objectKeypoints, objectDescriptors, imageKeypoints, imageDescriptors, ptpairs );
#endif


// Store keypoints
	(*point_pairs) = (int*)calloc(ptpairs.size(), sizeof(int));
	(*total_pairs) = ptpairs.size() / 2;
	
	
    for(int i = 0; i < (int)ptpairs.size(); i++)
    {
		(*point_pairs)[i] = ptpairs[i];
    }



    n = (int)(ptpairs.size()/2);
    if( n < 4 )
        return 0;

    pt1.resize(n);
    pt2.resize(n);
    for( i = 0; i < n; i++ )
    {
        pt1[i] = ((CvSURFPoint*)cvGetSeqElem(objectKeypoints,ptpairs[i*2]))->pt;
        pt2[i] = ((CvSURFPoint*)cvGetSeqElem(imageKeypoints,ptpairs[i*2+1]))->pt;
    }

    _pt1 = cvMat(1, n, CV_32FC2, &pt1[0] );
    _pt2 = cvMat(1, n, CV_32FC2, &pt2[0] );
    if( !cvFindHomography( &_pt1, &_pt2, &_h, CV_RANSAC, 5 ))
        return 0;

    for( i = 0; i < 4; i++ )
    {
        double x = src_corners[i].x, y = src_corners[i].y;
        double Z = 1./(h[6]*x + h[7]*y + h[8]);
        double X = (h[0]*x + h[1]*y + h[2])*Z;
        double Y = (h[3]*x + h[4]*y + h[5])*Z;
        dst_corners[i] = cvPoint(cvRound(X), cvRound(Y));
    }

    return 1;
}



void detect_path()
{
	if(!opencv_initialized)
	{
		opencv_initialized = 1;
	    initModule_nonfree();
		storage = cvCreateMemStorage(0);
		object_image = cvCreateImage( 
			cvSize(vision.working_w, vision.working_h), 
			8, 
			1);
		scene_image = cvCreateImage( 
			cvSize(vision.working_w, vision.working_h), 
			8, 
			1);

// read the 1st batch of reference frames
		ref_fd = fopen(REF_PATH, "r");
		if(!ref_fd) 
		{
			printf("Couldn't open %s\n", REF_PATH);
			exit(1);
		}

		int i;
		for(i = 0; i < REF_FRAMES; i++)
		{
			ref_y[i] = (uint8_t*)malloc(vision.working_w * vision.working_h);
			ref_u[i] = (uint8_t*)malloc(vision.working_w * vision.working_h);
			ref_v[i] = (uint8_t*)malloc(vision.working_w * vision.working_h);
			
			read_file_frame(ref_fd, ref_y[i], ref_u[i], ref_v[i]);
		}
	}

// convert image to opencv input
	CvSURFParams params = cvSURFParams(500, 1);
	
	cvSetImageROI( object_image, cvRect( 0, 0, vision.working_w, vision.working_h ) );
	cvSetImageROI( scene_image, cvRect( 0, 0, vision.working_w, vision.working_h ) );

	memcpy(scene_image->imageData, vision.y_buffer, vision.working_w * vision.working_h);


	int ref_frame = 0;
	for(ref_frame = 0; ref_frame < REF_FRAMES; ref_frame++)
	{
		memcpy(object_image->imageData, ref_y[ref_frame], vision.working_w * vision.working_h);

		if(object_keypoints) cvClearSeq(object_keypoints);
		if(object_descriptors) cvClearSeq(object_descriptors);

		cvExtractSURF(object_image, 
			0, 
			&object_keypoints, 
			&object_descriptors, 
			storage, 
			params,
			0);
		if(scene_keypoints) cvClearSeq(scene_keypoints);
		if(scene_descriptors) cvClearSeq(scene_descriptors);
		cvExtractSURF(scene_image, 
			0, 
			&scene_keypoints, 
			&scene_descriptors, 
			storage, 
			params,
			0);

		int *point_pairs = 0;
		int total_pairs = 0;
		CvPoint src_corners[4] = 
		{
			{ 0, 0 }, 
			{ vision.working_w, 0 }, 
			{ vision.working_w, vision.working_h }, 
			{ 0, vision.working_h }
		};

		CvPoint dst_corners[4] = 
		{
			{ 0, 0 },
			{ 0, 0 },
			{ 0, 0 },
			{ 0, 0 }
		};

	//printf("FindObjectMain::process_surf %d\n", __LINE__);
		if(scene_keypoints->total &&
			object_keypoints->total &&
				locatePlanarObject(object_keypoints, 
					object_descriptors, 
					scene_keypoints, 
					scene_descriptors, 
					src_corners, 
					dst_corners,
					&point_pairs,
					&total_pairs))
		{
			printf("detect_path %d total_pairs=%d\n", __LINE__, total_pairs);


	// draw debugging images
			for(int i = 0; i < vision.working_h; i++)
			{
	// object
				memcpy(vision.out_y + i * vision.output_w,
					vision.y_buffer + i * vision.working_w,
					vision.working_w);
				memcpy(vision.out_u + i * vision.output_w,
					vision.u_buffer + i * vision.working_w,
					vision.working_w);
				memcpy(vision.out_v + i * vision.output_w,
					vision.v_buffer + i * vision.working_w,
					vision.working_w);

	// reference
				memcpy(vision.out_y + vision.working_w + i * vision.output_w,
					ref_y[ref_frame] + i * vision.working_w,
					vision.working_w);
				memcpy(vision.out_u + vision.working_w + i * vision.output_w,
					ref_u[ref_frame] + i * vision.working_w,
					vision.working_w);
				memcpy(vision.out_v + vision.working_w + i * vision.output_w,
					ref_v[ref_frame] + i * vision.working_w,
					vision.working_w);
			}

			for(int i = 0; i < total_pairs; i++)
			{
        		CvSURFPoint* r1 = (CvSURFPoint*)cvGetSeqElem( object_keypoints, point_pairs[i * 2] );
        		CvSURFPoint* r2 = (CvSURFPoint*)cvGetSeqElem( scene_keypoints, point_pairs[i * 2 + 1] );


				int size = r2->size * 1.2 / 9 * 2;
	// object
				draw_rect(r1->pt.x - size, 
  					r1->pt.y - size, 
  					r1->pt.x + size, 
 					r1->pt.y + size);
	// scene
				draw_rect(r2->pt.x + vision.working_w - size, 
  					r2->pt.y - size, 
  					r2->pt.x + vision.working_w + size, 
 					r2->pt.y + size);
				draw_line(r1->pt.x, 
  					r1->pt.y,
					r2->pt.x + vision.working_w, 
  					r2->pt.y);
			}
			
			compress_jpeg();
// best match has most parallel lines of distance closest to working_w
//			printf("detect_path %d\n", __LINE__);
		}

		if(point_pairs) free(point_pairs);
	}

}


#endif // USE_OPENCV



