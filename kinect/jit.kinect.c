/* 
	Copyright 2001 - Cycling '74
	Joshua Kit Clayton jkc@cycling74.com	
	Andrew Roth aroth21@yorku.ca

	based on libfreenect - an open source Kinect driver
 
 Copyright (C) 2010  Hector Martin "marcan" <hector@marcansoft.com>
 
 This code is licensed to you under the terms of the GNU GPL, version 2 or version 3;
 see:
 http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt
 http://www.gnu.org/licenses/gpl-3.0.txt
 */

#include "jit.common.h"

//for kinect

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <inttypes.h>
#include <pthread.h>


#include <libusb.h>
#include "libfreenect.h"

#include <math.h>


//open and close messages
#define CONNECT "open"
#define DISCONNECT "close"

//from GLview example
libusb_device_handle *dev;

pthread_t gl_thread;
volatile int die = 0;

int g_argc;
char **g_argv;

int window;

pthread_mutex_t gl_backbuf_mutex = PTHREAD_MUTEX_INITIALIZER;

uint8_t gl_depth_front[640*480*4];
uint8_t gl_depth_back[640*480*4];

uint8_t gl_rgb_front[640*480*4];
uint8_t gl_rgb_back[640*480*4];

GLuint gl_depth_tex;
GLuint gl_rgb_tex;

uint16_t t_gamma[2048];


pthread_cond_t gl_frame_cond = PTHREAD_COND_INITIALIZER;
int got_frames = 0;



//=============================================================================
// Implementation
//=============================================================================


typedef struct _jit_kinect 
{
	t_object				ob;
	long					mapcount;
	long					map[JIT_MATRIX_MAX_DIMCOUNT];
	t_symbol				*Uname;
	t_symbol				*matrix_name;
	int						CamisOpen;
	
} t_jit_kinect;


//kinect


void *_jit_kinect_class;

t_jit_kinect *jit_kinect_new(void);
void jit_kinect_free(t_jit_kinect *x);
t_jit_err jit_kinect_matrix_calc(t_jit_kinect *x, void *inputs, void *outputs);
t_jit_err jit_kinect_init(void); 
void jit_kinect_open(t_jit_kinect *x,  t_symbol *s, long argc, t_atom *argv);
void OpenCamera(void);
void rgbimg(uint8_t *buf, int width, int height);
void depthimg(uint16_t *buf, int width, int height);

t_jit_err jit_kinect_init(void) 
{
	
	long attrflags=0;
	void *mop, *o;
	
	_jit_kinect_class = jit_class_new("jit_kinect",(method)jit_kinect_new,(method)jit_kinect_free,
		sizeof(t_jit_kinect),0L); 

	//add mop
	mop = jit_object_new(_jit_sym_jit_mop,1,2); 
	jit_class_addadornment(_jit_kinect_class,mop);
	
	jit_class_addmethod(_jit_kinect_class, (method)jit_kinect_open, 		"open", 		A_GIMME, 0L);
	jit_class_addmethod(_jit_kinect_class, (method)jit_kinect_open, 		"close", 		A_GIMME, 0L);
	o=jit_object_method(mop,_jit_sym_getoutput,1);
	
	//add methods
	jit_class_addmethod(_jit_kinect_class, (method)jit_kinect_matrix_calc, 		"matrix_calc", 		A_CANT, 0L);
	
	//add attributes 
	attrflags = JIT_MATRIX_DATA_REFERENCE;
	
	jit_class_register(_jit_kinect_class);

	return JIT_ERR_NONE;
}

t_jit_err jit_kinect_matrix_calc(t_jit_kinect *x, void *inputs, void *outputs)
{	
	t_jit_err err=JIT_ERR_NONE;
	t_jit_matrix_info out_minfo, tmp_minfo;
	t_matrix_conv_info mcinfo;
	
	void *out_matrix = NULL;
	void *tmp_matrix = NULL;
	char* out_bp = NULL; 
	char* tmp_bp = NULL;
	long out_savelock;

	post ("do something here");
	
	out_matrix 	= jit_object_method(outputs,_jit_sym_getindex,0);
	if (x&&out_matrix) {
		
		if (dev){
			static int fcnt = 0;
			pthread_mutex_lock(&gl_backbuf_mutex);
		
			while (got_frames < 2) {
				pthread_cond_wait(&gl_frame_cond, &gl_backbuf_mutex);
			}
		
			memcpy(gl_depth_front, gl_depth_back, sizeof(gl_depth_back));
			memcpy(gl_rgb_front, gl_rgb_back, sizeof(gl_rgb_back));
			out_savelock = (long) jit_object_method(out_matrix, _jit_sym_lock, 1);
			jit_object_method(out_matrix,_jit_sym_getdata, &out_bp);
			jit_object_method(out_matrix,_jit_sym_getinfo, &out_minfo);
			if (!out_bp) { err=JIT_ERR_INVALID_OUTPUT; goto out;}
			//tmp_minfo.size = frame->allocated_image_bytes;
			tmp_minfo.dim[0] = 640;
			tmp_minfo.dim[1] = 480;
			tmp_minfo.dimstride[0] = 1;
			tmp_minfo.dimstride[1] = 1;
			tmp_minfo.dimcount = 2;
			tmp_minfo.planecount = 4;
			tmp_minfo.type = _jit_sym_char;
			tmp_minfo.flags = JIT_MATRIX_DATA_REFERENCE|JIT_MATRIX_DATA_FLAGS_USE;
		
			mcinfo.flags = JIT_MATRIX_CONVERT_DSTDIM;
			mcinfo.srcdimstart[0] 	= 0;
			mcinfo.srcdimend[0] 	= tmp_minfo.dim[0]-1;
			mcinfo.srcdimstart[1] 	= 0;
			mcinfo.srcdimend[1] 	= tmp_minfo.dim[1]-1;
			out_minfo = tmp_minfo;
			
			//frame->image to out_bp
			//jit_object_method(out_matrix,_jit_sym_setinfo,&out_minfo);
			jit_object_method(out_bp, gl_depth_front);
			
			got_frames = 0;
			pthread_mutex_unlock(&gl_backbuf_mutex);
			
			
			
	/*
		
		 
		 if (camera) {
				err=dc1394_capture_dequeue(camera, DC1394_CAPTURE_POLICY_WAIT, &frame);
				if (err != 0) post("Could not dequeue a frame: error code %d\n", err);
				out_savelock = (long) jit_object_method(out_matrix, _jit_sym_lock, 1);
				jit_object_method(out_matrix,_jit_sym_getdata, &out_bp);
				jit_object_method(out_matrix,_jit_sym_getinfo, &out_minfo);
			if (!out_bp) { err=JIT_ERR_INVALID_OUTPUT; goto out;}
			
				jit_object_method(tmp_matrix,_jit_sym_getdata, &tmp_bp);
				jit_object_method(tmp_matrix,_jit_sym_getinfo, &tmp_minfo);
			if (!tmp_bp) { err=JIT_ERR_INVALID_OUTPUT; goto out;}
		
				tmp_minfo.size = frame->allocated_image_bytes;
				tmp_minfo.dim[0] = 1024;
				tmp_minfo.dim[1] = 768*2;
				tmp_minfo.dimstride[0] = 1;
				tmp_minfo.dimstride[1] = 1;
				tmp_minfo.dimcount = 2;
				tmp_minfo.planecount = 3;
				tmp_minfo.type = _jit_sym_char;
				tmp_minfo.flags = JIT_MATRIX_DATA_REFERENCE|JIT_MATRIX_DATA_FLAGS_USE;
			
				mcinfo.flags = JIT_MATRIX_CONVERT_DSTDIM;
				mcinfo.srcdimstart[0] 	= 0;
				mcinfo.srcdimend[0] 	= tmp_minfo.dim[0]-1;
				mcinfo.srcdimstart[1] 	= 0;
				mcinfo.srcdimend[1] 	= tmp_minfo.dim[1]-1;
				out_minfo = tmp_minfo;
				//jit_object_method(out_matrix,_jit_sym_getdata,&mcinfo); 
					
				assert( out_minfo.dim[0] == 1024 && out_minfo.dim[1] == 1536 && out_minfo.dimcount == 2 && out_minfo.planecount == 3 );
				post("%d || %d", out_minfo.dim[0], out_minfo.dim[1]);
		
				err=dc1394_deinterlace_stereo(frame->image, tmp_bp, out_minfo.dim[0], out_minfo.dim[1]);
				if (err != 0) post("Could not dequeue a frame: error code %d\n", err);
		
				err=dc1394_bayer_decoding_8bit(tmp_bp, out_bp, out_minfo.dim[0], out_minfo.dim[1]*2, DC1394_COLOR_FILTER_RGGB, DC1394_BAYER_METHOD_NEAREST);
				if (err != 0) post("Error from bayer = %d\n", err);
		
				err = (t_jit_err) jit_object_method(out_matrix,_jit_sym_setinfo,&out_minfo);

				// release frame
				err=dc1394_capture_enqueue(camera, frame);
				if (err != 0) post("Could not enqueue a frame\n", err);
				if (err) goto out;
	 */
			} else {
				post ("Kinect not Open!"); 
			}
		} else {
		return JIT_ERR_INVALID_PTR;
	}
out:
	if (out_matrix)
	jit_object_method(out_matrix,_jit_sym_lock,out_savelock);
	//if (tmp_matrix) jit_object_free(tmp_matrix);
	return err; 
	
}

t_jit_kinect *jit_kinect_new(void)
{
	t_jit_kinect *x;
		
	if (x=(t_jit_kinect *)jit_object_alloc(_jit_kinect_class)) {
		
	} else {
		x = NULL;
	}	
	return x;
}

void jit_kinect_free(t_jit_kinect *x)
{
	
	libusb_release_interface(dev, 0);
	post("interface released.");
	if(dev){
		libusb_close(dev);
		post("libusb closed.");
		
	}
	libusb_exit(NULL);
	post("libusb exited successfully.");
		//nada
	/*
	 Nov 16, crashes, pthread likely not initialized yet.
	pthread_exit(NULL);	
	*/
	
}

void jit_kinect_open(t_jit_kinect *x,  t_symbol *s, long argc, t_atom *argv)
{
	//long ia;
	//t_atom* ap;
	char* con_1 = CONNECT;
		if (0==strcmp(s->s_name, con_1)) {
			post("opening...");
			OpenCamera();
		}
	
	
	/*
	for (ia = 0, ap = argv; ia < argc; ia++, ap++) {       // increment ap each time to get to the next atom
		switch (argv[ia].a_type) {
			case A_LONG:
				post("Long: %ld: %ld",ia+1,argv[ia].a_w.w_long);
				break;
			case A_FLOAT:
				//post("Float %ld: %.2f",ia+1,argv[ia].a_w.w_float);
				if (ia < 2)
				//patt_center[ia] = argv[ia].a_w.w_float;
				break;
			case A_SYM:
				//post("Sym %ld: %s",ia+1, argv[ia].a_w.w_sym->s_name);
				post("%s", s->s_name);
				if (0==strcmp(s->s_name, con_1)) {
					post("opening...");
					OpenCamera();
				}
				break;
			default:
				post("%ld: unknown atom type (%ld)", ia+1, argv[ia].a_w.w_sym->s_name);
				break;
	
		
		}
	}
	 */
}

void OpenCamera(){
	post("attempting to open camera");
	libusb_init(NULL);
	
	int res;

	post("Kinect camera test\n");
	
	int i;
	for (i=0; i<2048; i++) {
		float v = i/2048.0;
		v = powf(v, 3)* 6;
		t_gamma[i] = v*6*256;
	}
	 
	
	//g_argc = argc;
	//g_argv = argv;
	
	dev = libusb_open_device_with_vid_pid(NULL, 0x45e, 0x2ae);
	
	if (!dev) {
		post("Could not open device\n");
		return;
	}
		
	libusb_claim_interface(dev, 0);
		post("device is %d\n", libusb_get_device_address(libusb_get_device(dev)));
		
	cams_init(dev, depthimg, rgbimg);
}

void rgbimg(uint8_t *buf, int width, int height)
{
	
	int i;
	
	pthread_mutex_lock(&gl_backbuf_mutex);
	memcpy(gl_rgb_back, buf, width*height*3);
	got_frames++;
	pthread_cond_signal(&gl_frame_cond);
	pthread_mutex_unlock(&gl_backbuf_mutex);
}

void depthimg(uint16_t *buf, int width, int height)
{
	/*
	 FILE *f = fopen("depth.bin", "w");
	 fwrite(depth_frame, 640*480, 2, f);
	 fclose(f);
	*/
	int i;
	
	pthread_mutex_lock(&gl_backbuf_mutex);
	for (i=0; i<640*480; i++) {
		int pval = t_gamma[buf[i]];
		int lb = pval & 0xff;
		switch (pval>>8) {
			case 0:
				gl_depth_back[3*i+0] = 255;
				gl_depth_back[3*i+1] = 255-lb;
				gl_depth_back[3*i+2] = 255-lb;
				break;
			case 1:
				gl_depth_back[3*i+0] = 255;
				gl_depth_back[3*i+1] = lb;
				gl_depth_back[3*i+2] = 0;
				break;
			case 2:
				gl_depth_back[3*i+0] = 255-lb;
				gl_depth_back[3*i+1] = 255;
				gl_depth_back[3*i+2] = 0;
				break;
			case 3:
				gl_depth_back[3*i+0] = 0;
				gl_depth_back[3*i+1] = 255;
				gl_depth_back[3*i+2] = lb;
				break;
			case 4:
				gl_depth_back[3*i+0] = 0;
				gl_depth_back[3*i+1] = 255-lb;
				gl_depth_back[3*i+2] = 255;
				break;
			case 5:
				gl_depth_back[3*i+0] = 0;
				gl_depth_back[3*i+1] = 0;
				gl_depth_back[3*i+2] = 255-lb;
				break;
			default:
				gl_depth_back[3*i+0] = 0;
				gl_depth_back[3*i+1] = 0;
				gl_depth_back[3*i+2] = 0;
				break;
		}
	}
	got_frames++;
	pthread_cond_signal(&gl_frame_cond);
	pthread_mutex_unlock(&gl_backbuf_mutex);
}


