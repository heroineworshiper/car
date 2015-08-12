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

// define whether to use approximate nearest-neighbor search or brute force
//#define USE_FLANN

// Use full color to match frames
//#define MATCH_COLOR

//#define GLOBAL_MOTION

#define SQR(x) ((x) * (x))

// detect path by matching images with OpenCV
using namespace cv;
using namespace std;

IplImage *current_image = 0;
IplImage *ref_image = 0;
CvMemStorage *storage = 0;

CvSeq *y_keypoints = 0;
CvSeq *y_descriptors = 0;
CvSeq *u_keypoints = 0;
CvSeq *u_descriptors = 0;
CvSeq *v_keypoints = 0;
CvSeq *v_descriptors = 0;
int opencv_initialized = 0;

// reference file
#define REF_FRAMES 100
typedef struct
{
	unsigned char *y;
	unsigned char *u;
	unsigned char *v;
	CvSeq *y_keypoints;
	CvSeq *y_descriptors;
	CvSeq *u_keypoints;
	CvSeq *u_descriptors;
	CvSeq *v_keypoints;
	CvSeq *v_descriptors;
// must save the keypoint pairs in the refs in case they're used for
// motion
	int *y_pairs;
	int y_total;
	int *u_pairs;
	int u_total;
	int *v_pairs;
	int v_total;

} ref_t;
FILE *ref_fd;
ref_t refs[REF_FRAMES];
int refs_filled = 0;

// downscaled images for motion
unsigned char *motion_ref_y = 0;
unsigned char *motion_ref_u = 0;
unsigned char *motion_ref_v = 0;
unsigned char *motion_current_y = 0;
unsigned char *motion_current_u = 0;
unsigned char *motion_current_v = 0;
int motion_scale = 2;

//PyrLkRobustMotionEstimator *motionEstimator = 0;


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

//     pt1.resize(n);
//     pt2.resize(n);
//     for( i = 0; i < n; i++ )
//     {
//         pt1[i] = ((CvSURFPoint*)cvGetSeqElem(objectKeypoints,ptpairs[i*2]))->pt;
//         pt2[i] = ((CvSURFPoint*)cvGetSeqElem(imageKeypoints,ptpairs[i*2+1]))->pt;
//     }
// 
//     _pt1 = cvMat(1, n, CV_32FC2, &pt1[0] );
//     _pt2 = cvMat(1, n, CV_32FC2, &pt2[0] );
//     if( !cvFindHomography( &_pt1, &_pt2, &_h, CV_RANSAC, 5 ))
//         return 0;
// 
//     for( i = 0; i < 4; i++ )
//     {
//         double x = src_corners[i].x, y = src_corners[i].y;
//         double Z = 1./(h[6]*x + h[7]*y + h[8]);
//         double X = (h[0]*x + h[1]*y + h[2])*Z;
//         double Y = (h[3]*x + h[4]*y + h[5])*Z;
//         dst_corners[i] = cvPoint(cvRound(X), cvRound(Y));
//     }

    return 1;
}



void detect_path()
{
	if(!opencv_initialized)
	{
		opencv_initialized = 1;
		
		bzero(refs, sizeof(refs));
		refs_filled = 0;
	    initModule_nonfree();
		storage = cvCreateMemStorage(0);
		current_image = cvCreateImage( 
			cvSize(vision.working_w, vision.working_h), 
			8, 
			1);
		ref_image = cvCreateImage( 
			cvSize(vision.working_w, vision.working_h), 
			8, 
			1);

// read the 1st batch of reference frames
		ref_fd = fopen(vision.ref_path, "r");
		if(!ref_fd) 
		{
			printf("Couldn't open %s\n", vision.ref_path);
			exit(1);
		}

		int i;
		for(i = 0; i < REF_FRAMES; i++)
		{
			refs[i].y = (uint8_t*)malloc(vision.working_w * vision.working_h);
			refs[i].u = (uint8_t*)malloc(vision.working_w * vision.working_h);
			refs[i].v = (uint8_t*)malloc(vision.working_w * vision.working_h);
		}
	}

// fill the ref frames
	while(refs_filled < REF_FRAMES)
	{
		ref_t *dst = &refs[refs_filled];
// reset the ref but keep image allocations
		if(dst->y_keypoints) cvClearSeq(dst->y_keypoints);
		if(dst->u_keypoints) cvClearSeq(dst->u_keypoints);
		if(dst->v_keypoints) cvClearSeq(dst->v_keypoints);
		if(dst->y_descriptors) cvClearSeq(dst->y_descriptors);
		if(dst->u_descriptors) cvClearSeq(dst->u_descriptors);
		if(dst->v_descriptors) cvClearSeq(dst->v_descriptors);
		dst->y_keypoints = 0;
		dst->u_keypoints = 0;
		dst->v_keypoints = 0;
		dst->y_descriptors = 0;
		dst->u_descriptors = 0;
		dst->v_descriptors = 0;

		read_file_frame(ref_fd, dst->y, dst->u, dst->v);
		refs_filled++;
	}

// convert image to opencv input
	CvSURFParams params = cvSURFParams(500, 1);

	cvSetImageROI( current_image, cvRect( 0, 0, vision.working_w, vision.working_h ) );
	cvSetImageROI( ref_image, cvRect( 0, 0, vision.working_w, vision.working_h ) );

	if(y_keypoints) cvClearSeq(y_keypoints);
	if(u_keypoints) cvClearSeq(u_keypoints);
	if(v_keypoints) cvClearSeq(v_keypoints);
	if(y_descriptors) cvClearSeq(y_descriptors);
	if(u_descriptors) cvClearSeq(u_descriptors);
	if(v_descriptors) cvClearSeq(v_descriptors);
	y_keypoints = 0;
	u_keypoints = 0;
	v_keypoints = 0;
	y_descriptors = 0;
	u_descriptors = 0;
	v_descriptors = 0;

	memcpy(current_image->imageData, vision.y_buffer, vision.working_w * vision.working_h);
	cvExtractSURF(current_image, 
		0, 
		&y_keypoints, 
		&y_descriptors, 
		storage, 
		params,
		0);
#ifdef MATCH_COLOR
	memcpy(current_image->imageData, vision.u_buffer, vision.working_w * vision.working_h);
	cvExtractSURF(current_image, 
		0, 
		&u_keypoints, 
		&u_descriptors, 
		storage, 
		params,
		0);
	memcpy(current_image->imageData, vision.v_buffer, vision.working_w * vision.working_h);
	cvExtractSURF(current_image, 
		0, 
		&v_keypoints, 
		&v_descriptors, 
		storage, 
		params,
		0);
#endif // MATCH_COLOR

	int ref_frame = 0;
	int max_keypoints = 0;
	int max_frame = 0;
	for(ref_frame = 0; ref_frame < REF_FRAMES; ref_frame++)
	{
		if(refs[ref_frame].y_pairs) free(refs[ref_frame].y_pairs);
		if(refs[ref_frame].u_pairs) free(refs[ref_frame].u_pairs);
		if(refs[ref_frame].v_pairs) free(refs[ref_frame].v_pairs);
		refs[ref_frame].y_pairs = 0;
		refs[ref_frame].y_total = 0;
		refs[ref_frame].u_pairs = 0;
		refs[ref_frame].u_total = 0;
		refs[ref_frame].v_pairs = 0;
		refs[ref_frame].v_total = 0;

// SURF scan the ref
		if(!refs[ref_frame].y_keypoints)
		{
			memcpy(ref_image->imageData, refs[ref_frame].y, vision.working_w * vision.working_h);
			cvExtractSURF(ref_image, 
				0, 
				&refs[ref_frame].y_keypoints, 
				&refs[ref_frame].y_descriptors, 
				storage, 
				params,
				0);
#ifdef MATCH_COLOR
			memcpy(ref_image->imageData, refs[ref_frame].u, vision.working_w * vision.working_h);
			cvExtractSURF(ref_image, 
				0, 
				&refs[ref_frame].u_keypoints, 
				&refs[ref_frame].u_descriptors, 
				storage, 
				params,
				0);
			memcpy(ref_image->imageData, refs[ref_frame].v, vision.working_w * vision.working_h);
			cvExtractSURF(ref_image, 
				0, 
				&refs[ref_frame].v_keypoints, 
				&refs[ref_frame].v_descriptors, 
				storage, 
				params,
				0);
#endif // MATCH_COLOR
		}

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
		if(refs[ref_frame].y_keypoints->total &&
			y_keypoints->total)
		{
			locatePlanarObject(y_keypoints, 
				y_descriptors, 
				refs[ref_frame].y_keypoints, 
				refs[ref_frame].y_descriptors, 
				src_corners, 
				dst_corners,
				&refs[ref_frame].y_pairs,
				&refs[ref_frame].y_total);  
		}

#ifdef MATCH_COLOR
		if(refs[ref_frame].u_keypoints->total &&
			u_keypoints->total)
		{
			locatePlanarObject(u_keypoints, 
				u_descriptors, 
				refs[ref_frame].u_keypoints, 
				refs[ref_frame].u_descriptors, 
				src_corners, 
				dst_corners,
				&refs[ref_frame].u_pairs,
				&refs[ref_frame].u_total);
		}
		if(refs[ref_frame].v_keypoints->total &&
			v_keypoints->total)
		{
			locatePlanarObject(v_keypoints, 
				v_descriptors, 
				refs[ref_frame].v_keypoints, 
				refs[ref_frame].v_descriptors, 
				src_corners, 
				dst_corners,
				&refs[ref_frame].v_pairs,
				&refs[ref_frame].v_total);
		}
#endif // MATCH_COLOR


		if(refs[ref_frame].y_total + refs[ref_frame].u_total + refs[ref_frame].v_total > max_keypoints)
		{
			max_frame = ref_frame;
			max_keypoints = refs[ref_frame].y_total + refs[ref_frame].u_total + refs[ref_frame].v_total;
		}
// printf("detect_path %d total_pairs=%d\n", 
// 	__LINE__, 
// 	refs[ref_frame].total_pairs);


	}


// Optical flow
// 	if(!motionEstimator) 
// 	{
// 		motionEstimator = new PyrLkRobustMotionEstimator();
// 		motionEstimator->setMotionModel(TRANSLATION);
// 		
// 	}
	

// get global motion
	int dx = 0;
	int dy = 0;
// 	double avg_current_x = 0;
// 	double avg_current_y = 0;
// 	double avg_ref_x = 0;
// 	double avg_ref_y = 0;
// 	if(refs[max_frame].y_total > 0)
// 	{
// 		for(int i = 0; i < refs[max_frame].y_total; i++)
// 		{
// 			CvSURFPoint* r1 = (CvSURFPoint*)cvGetSeqElem( y_keypoints, refs[max_frame].y_pairs[i * 2] );
// 			CvSURFPoint* r2 = (CvSURFPoint*)cvGetSeqElem( refs[max_frame].y_keypoints, refs[max_frame].y_pairs[i * 2 + 1] );
// 			avg_current_x += r1->pt.x;
// 			avg_current_y += r1->pt.y;
// 			avg_ref_x += r2->pt.x;
// 			avg_ref_y += r2->pt.y;
// 		}
// 		avg_current_x /= refs[max_frame].y_total;
// 		avg_current_y /= refs[max_frame].y_total;
// 		avg_ref_x /= refs[max_frame].y_total;
// 		avg_ref_y /= refs[max_frame].y_total;
// 		dx = avg_ref_x - avg_current_x;
// 		dy = avg_ref_y - avg_current_y;
// 	}



/*
 *  	for(int i = 0; i < refs[max_frame].total_pairs; i++)
 *  	{
 *          CvSURFPoint* r1 = (CvSURFPoint*)cvGetSeqElem( current_keypoints, refs[max_frame].point_pairs[i * 2] );
 *          CvSURFPoint* r2 = (CvSURFPoint*)cvGetSeqElem( refs[max_frame].keypoints, refs[max_frame].point_pairs[i * 2 + 1] );
 * 		 
 * 		 dx += r2->pt.x - r1->pt.x;
 * 		 dy += r2->pt.y - r1->pt.y;
 * 	}
 * 
 * 	dx /= refs[max_frame].total_pairs;
 * 	dy /= refs[max_frame].total_pairs;
 */


#ifdef GLOBAL_MOTION
	int search_step = 2;

// make downsampled images
	if(!motion_ref_y)
	{
		motion_ref_y = malloc(vision.working_w / 2 * vision.working_h / 2);
		motion_ref_u = malloc(vision.working_w / 2 * vision.working_h / 2);
		motion_ref_v = malloc(vision.working_w / 2 * vision.working_h / 2);
		motion_current_y = malloc(vision.working_w / 2 * vision.working_h / 2);
		motion_current_u = malloc(vision.working_w / 2 * vision.working_h / 2);
		motion_current_v = malloc(vision.working_w / 2 * vision.working_h / 2);
	}


	for(int i = 0; i < vision.working_h / motion_scale; i++)
	{
		unsigned char *ref_y = refs[max_frame].y + i * vision.working_w;
		unsigned char *ref_u = refs[max_frame].u + i * vision.working_w;
		unsigned char *ref_v = refs[max_frame].v + i * vision.working_w;
		unsigned char *current_y = vision.y_buffer + i * vision.working_w;
		unsigned char *current_u = vision.u_buffer + i * vision.working_w;
		unsigned char *current_v = vision.v_buffer + i * vision.working_w;
		unsigned char *dst_

		for(int j = 0; j < current_w; j++)
		{
// best results from squared diff of color cube
			int y_value = *ref_y - *current_y;
			y_value *= y_value;
			ref_y++;
			current_y++;

			int u_value = *ref_u - *current_u;
			u_value *= u_value;
			ref_u++;
			current_u++;

			int v_value = *ref_v - *current_v;
			v_value *= v_value;
			ref_v++;
			current_v++;

			abs_diff += y_value + u_value + v_value;
		}
	}

//	int block_w = vision.working_w;
//	int block_h = vision.working_h;
//	int search_x1 = -vision.working_w / 4;
//	int search_y1 = -vision.working_h / 4;
//	int search_x2 = vision.working_w / 4;
//	int search_y2 = vision.working_h / 4;
	int block_w = vision.working_w / 2;
	int block_h = vision.working_h / 2;
	int search_x1 = 0;
	int search_y1 = 0;
	int search_x2 = vision.working_w - block_w;
	int search_y2 = vision.working_h - block_h;
	int global_search_x2 = search_x2;
	int global_search_y2 = search_y2;
	int ref_x = (vision.working_w - block_w) / 2;
	int ref_y = (vision.working_h - block_h) / 2;
	double diff_history[search_x2 * search_y2];
	int history_used[search_x2 * search_y2];
	bzero(history_used, sizeof(history_used));
	
	while(search_step >= 1)
	{
		double min_diff = 0x7fffffff;
		int best_x;
		int best_y;
		for(int search_x = search_x1; search_x < search_x2; search_x += search_step)
		{
			for(int search_y = search_y1; search_y < search_y2; search_y += search_step)
			{
				double avg_diff = 0;
				int history_index = search_y * global_search_x2 + search_x;
				if(history_used[history_index])
				{
					avg_diff = diff_history[history_index];
				}
				else
				{
					int64_t abs_diff = 0;

	// generate clamped extents of areas to compare
	// area in current frame
					int current_x1 = (vision.working_w - block_w) / 2;
					int current_y1 = (vision.working_h - block_h) / 2;
					int current_x2 = current_x1 + block_w;
					int current_y2 = current_y1 + block_h;
	// area in ref frame
					int ref_x1 = search_x;
					int ref_y1 = search_y;
					int ref_x2 = ref_x1 + block_w;
					int ref_y2 = ref_y1 + block_h;

					if(ref_x1 < 0)
					{
						current_x1 += -ref_x1;
						ref_x1 = 0;
					}

					if(ref_y1 < 0)
					{
						current_y1 += -ref_y1;
						ref_y1 = 0;
					}

					if(ref_x2 > vision.working_w)
					{
						current_x2 -= ref_x2 - vision.working_w;
						ref_x2 = vision.working_w;
					}

					if(ref_y2 > vision.working_h)
					{
						current_y2 -= ref_y2 - vision.working_h;
						ref_y2 = vision.working_h;
					}

					int current_w = current_x2 - current_x1;
					int current_h = current_y2 - current_y1;

	//printf("detect_path ref_y1=%d ref_y2=%d ref_x1=%d ref_x2=%d current_x1=%d current_x2=%d current_y1=%d current_y2=%d\n", 
	//ref_y1, ref_y2, ref_x1, ref_x2, current_x1, current_x2, current_y1, current_y2);
					for(int i = 0; i < current_h; i++)
					{
						unsigned char *ref_y = refs[max_frame].y + 
							(ref_y1 + i) * vision.working_w + 
							ref_x1;
						unsigned char *ref_u = refs[max_frame].u + 
							(ref_y1 + i) * vision.working_w + 
							ref_x1;
						unsigned char *ref_v = refs[max_frame].v + 
							(ref_y1 + i) * vision.working_w + 
							ref_x1;
						unsigned char *current_y = vision.y_buffer +
							(current_y1 + i) * vision.working_w +
							current_x1;
						unsigned char *current_u = vision.u_buffer +
							(current_y1 + i) * vision.working_w +
							current_x1;
						unsigned char *current_v = vision.v_buffer +
							(current_y1 + i) * vision.working_w +
							current_x1;

						for(int j = 0; j < current_w; j++)
						{
// best results from squared diff of color cube
							int y_value = *ref_y - *current_y;
							y_value *= y_value;
							ref_y++;
							current_y++;

							int u_value = *ref_u - *current_u;
							u_value *= u_value;
							ref_u++;
							current_u++;

							int v_value = *ref_v - *current_v;
							v_value *= v_value;
							ref_v++;
							current_v++;

							abs_diff += y_value + u_value + v_value;
						}
					}

					int pixels = current_w * current_h;
					avg_diff = (double)abs_diff / pixels;
					diff_history[history_index] = avg_diff;
					history_used[history_index] = 1;
				}
				
				
//printf("avg_diff=%f ref_x1=%d ref_y1=%d ref_x2=%d ref_y2=%d cur_x1=%d cur_y1=%d cur_x2=%d cur_y2=%d\n", 
//avg_diff, ref_x1, ref_y1, ref_x2, ref_y2, current_x1, current_y1, current_x2, current_y2);
				if(avg_diff < min_diff)
				{
					min_diff = avg_diff;
					best_x = search_x;
					best_y = search_y;
				}
			}
		}

		search_x1 = best_x - search_step;
		search_y1 = best_y - search_step;
		search_x2 = best_x + search_step;
		search_y2 = best_y + search_step;

		dx = best_x - ref_x;
		dy = best_y - ref_y;
		search_step /= 2;
	}
#endif // GLOBAL_MOTION

// draw best image, translated to database position
	bzero(vision.out_y, vision.output_w * vision.output_h);
	for(int i = 0; i < vision.working_h; i++)
	{
		int dst_y = i + dy;
		if(dst_y >= 0 && dst_y < vision.output_h)
		{
			int src_x1 = 0;
			int src_x2 = vision.working_w;
			int dst_x1 = dx;
			int dst_x2 = vision.working_w + dx;
			
			if(dst_x1 < 0)
			{
				src_x1 += -dst_x1;
				dst_x1 = 0;
			}
			
			if(dst_x2 > vision.working_w)
			{
				src_x2 -= dst_x2 - vision.working_w;
				dst_x2 = vision.working_w;
			}
			
			if(dst_x1 < dst_x2)
			{
// current image shifted in position
 				memcpy(vision.out_y + dst_y * vision.output_w + dst_x1,
 					vision.y_buffer + i * vision.working_w + src_x1,
 					dst_x2 - dst_x1);
 				memcpy(vision.out_u + dst_y * vision.output_w + dst_x1,
 					vision.u_buffer + i * vision.working_w + src_x1,
 					dst_x2 - dst_x1);
 				memcpy(vision.out_v + dst_y * vision.output_w + dst_x1,
 					vision.v_buffer + i * vision.working_w + src_x1,
 					dst_x2 - dst_x1);
				
			}
		}
	}	

// draw best image
 	for(int i = 0; i < vision.working_h; i++)
 	{
// // current
// 		memcpy(vision.out_y + i * vision.output_w,
// 			vision.y_buffer + i * vision.working_w,
// 			vision.working_w);
// 		memcpy(vision.out_u + i * vision.output_w,
// 			vision.u_buffer + i * vision.working_w,
// 			vision.working_w);
// 		memcpy(vision.out_v + i * vision.output_w,
// 			vision.v_buffer + i * vision.working_w,
// 			vision.working_w);
 
// reference
 		memcpy(vision.out_y + vision.working_w + i * vision.output_w,
 			refs[max_frame].y + i * vision.working_w,
 			vision.working_w);
 		memcpy(vision.out_u + vision.working_w + i * vision.output_w,
 			refs[max_frame].u + i * vision.working_w,
 			vision.working_w);
 		memcpy(vision.out_v + vision.working_w + i * vision.output_w,
 			refs[max_frame].v + i * vision.working_w,
 			vision.working_w);
	}


// draw keypoints
// 	for(int i = 0; i < refs[max_frame].total_pairs; i++)
// 	{
//         CvSURFPoint* r1 = (CvSURFPoint*)cvGetSeqElem( current_keypoints, refs[max_frame].point_pairs[i * 2] );
//         CvSURFPoint* r2 = (CvSURFPoint*)cvGetSeqElem( refs[max_frame].keypoints, refs[max_frame].point_pairs[i * 2 + 1] );
// 		int size = r2->size * 1.2 / 9 * 2;
// 
// // current
// 		draw_rect(r1->pt.x - size, 
//   			r1->pt.y - size, 
//   			r1->pt.x + size, 
//  			r1->pt.y + size);
// // ref
// 		draw_rect(r2->pt.x + vision.working_w - size, 
//   			r2->pt.y - size, 
//   			r2->pt.x + vision.working_w + size, 
//  			r2->pt.y + size);
// 		draw_line(r1->pt.x, 
//   			r1->pt.y,
// 			r2->pt.x + vision.working_w, 
//   			r2->pt.y);
// 	}
// 


//	compress_jpeg();
// best match has most pairs
	printf("detect_path %d max_pairs=%d max_frame=%d\n", 
		__LINE__, 
		max_keypoints,
		max_frame);

// advance refs if necessary
	if(max_frame >= REF_FRAMES / 2)
	{
		int drop = max_frame - REF_FRAMES / 2;
		ref_t tmp_refs[REF_FRAMES];
		
		memcpy(tmp_refs, refs, sizeof(ref_t) * drop);
		for(int i = 0; i < refs_filled - drop; i++)
		{
			refs[i] = refs[i + drop];
		}
		memcpy(&refs[refs_filled - drop], tmp_refs, sizeof(ref_t) * drop);
		
		refs_filled -= drop;
	}
}


#endif // USE_OPENCV



