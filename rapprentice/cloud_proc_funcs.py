import cloudprocpy
from rapprentice import berkeley_pr2, clouds, cv_plot_utils as cpu
import cv2, numpy as np
import skimage.morphology as skim
from scipy import interpolate

DEBUG_PLOTS=True
region_limits = np.array([(0.40,1.15),(-0.35,0.36),(100,1000)])
height_limits = np.array([0,0.25])  #np.array([0.06,0.18])
depth_limits = np.array([400,1200])
bgr_limits = np.array([(-1,-1,-1),(40,40,40)])

grid_x = np.linspace(region_limits[0,0],region_limits[0,1],3)
grid_y = np.linspace(region_limits[1,0],region_limits[1,1],3)
calib_adjustments = np.array([[(-60,-20,-35),(-30,5,-20),(-10,20,0)],
                              [(-60,-25,-10),(-10,-10,5),(0,5,20)],
                              [(-90,-60,-5),(-60,-20,15),(-40,-15,45)]])/1000.0

def extract_red(rgb, depth, T_w_k):
    """
    extract red points and downsample
    """

    hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)
    h = hsv[:,:,0]
    s = hsv[:,:,1]
    v = hsv[:,:,2]

    h_mask = (h<15) | (h>145)
    s_mask = (s > 30 )
    v_mask = (v > 100)
    red_mask = h_mask & s_mask & v_mask

    valid_mask = depth > 0

    xyz_k = clouds.depth_to_xyz(depth, berkeley_pr2.f)
    xyz_w = xyz_k.dot(T_w_k[:3,:3].T) + T_w_k[:3,3][None,None,:]

    z = xyz_w[:,:,2]
    z0 = xyz_k[:,:,2]

    region_mask = (xyz_w[:,:,0] > region_limits[0,0]) & (xyz_w[:,:,0] < region_limits[0,1]) & \
                  (xyz_w[:,:,1] > region_limits[1,0]) & (xyz_w[:,:,1] < region_limits[1,1])
    height_mask = (xyz_w[:,:,2] > height_limits[0]) & (xyz_w[:,:,2] < height_limits[1]) # TODO pass in parameter
    #height_mask = xyz_w[:,:,2] > .7 # TODO pass in parameter

    tmp_mask = red_mask & height_mask & valid_mask & region_mask
    for ii in [0,1,2]:
        xyz_w[tmp_mask,ii] += interpolate.interpn((grid_x,grid_y),calib_adjustments[:,:,ii],xyz_w[tmp_mask,0:2],
                                                  method='linear',bounds_error=False, fill_value=None)

    good_mask = skim.remove_small_objects(tmp_mask,min_size=10)
    #good_mask = skim.remove_small_objects(good_mask,min_size=64)

    if DEBUG_PLOTS:
        """
        imgs = [z/z.max(), h_mask.astype('uint8')*255, s_mask.astype('uint8')*255,
                v_mask.astype('uint8')*255, height_mask.astype('uint8')*255, good_mask.astype('uint8')*255]
        bigimg = cpu.tile_images(imgs, 2, 3)
        cv2.imshow("image filter results", bigimg)
        cv2.moveWindow("image filter results", 600, 0)
        """

        plotsize = np.array([700, int(700.0/rgb.shape[1]*rgb.shape[0])])
        cv2.imshow("rgb", cv2.resize(rgb,(plotsize[0], plotsize[1])))
        cv2.moveWindow("rgb", 790, 0)
        cv2.waitKey(1000)

        cv2.imshow("detected cable", cv2.resize(good_mask.astype('uint8')*255,(plotsize[0], plotsize[1])))
        cv2.moveWindow("detected cable", 790, 490)
        cv2.waitKey(1000)
        """
        cv2.imshow("z0",z0/z0.max())
        cv2.imshow("z",z/z.max())
        cv2.imshow("hue", h_mask.astype('uint8')*255)
        cv2.imshow("sat", s_mask.astype('uint8')*255)
        cv2.imshow("val", v_mask.astype('uint8')*255)
        cv2.imshow("final",good_mask.astype('uint8')*255)
        cv2.imshow("rgb", rgb)
        cv2.waitKey()
        """

    good_xyz = xyz_w[good_mask]

    return clouds.downsample(good_xyz, .025)


def extract_black(rgb, depth, T_w_k):
    """
    extract black points and downsample
    """

    b = rgb[:, :, 0]
    g = rgb[:, :, 1]
    r = rgb[:, :, 2]
    b_mask = (b > bgr_limits[0,0]) & (b < bgr_limits[1,0])
    g_mask = (g > bgr_limits[0,1]) & (g < bgr_limits[1,1])
    r_mask = (r > bgr_limits[0,2]) & (r < bgr_limits[1,2])
    black_mask = b_mask | g_mask | r_mask

    depth_mask = (depth > depth_limits[0]) & (depth < depth_limits[1])

    xyz_k = clouds.depth_to_xyz(depth, berkeley_pr2.f)
    xyz_w = xyz_k.dot(T_w_k[:3,:3].T) + T_w_k[:3,3][None,None,:]

    x,y = np.meshgrid(np.arange(1920), np.arange(1080))
    region_mask = (y > region_limits[2,0]) & (y < region_limits[2,1]) & \
                  (xyz_w[:,:,0] > region_limits[0,0]) & (xyz_w[:,:,0] < region_limits[0,1]) & \
                  (xyz_w[:,:,1] > region_limits[1,0]) & (xyz_w[:,:,1] < region_limits[1,1])

    tmp_mask = black_mask & depth_mask & region_mask
    for ii in [0,1,2]:
        xyz_w[tmp_mask,ii] += interpolate.interpn((grid_x,grid_y),calib_adjustments[:,:,ii],xyz_w[tmp_mask,0:2],
                                                  method='linear',bounds_error=False)

    z = xyz_w[:,:,2]
    z0 = xyz_k[:,:,2]

    height_mask = (xyz_w[:,:,2] > height_limits[0]) & (xyz_w[:,:,2] < height_limits[1]) # TODO pass in parameter

    good_mask = black_mask & height_mask & depth_mask & region_mask
    good_mask = skim.remove_small_objects(good_mask,min_size=10)

    if DEBUG_PLOTS:
        imgs = [z0/z0.max(), depth_mask.astype('uint8')*255, black_mask.astype('uint8')*255,
                height_mask.astype('uint8')*255, region_mask.astype('uint8')*255, good_mask.astype('uint8')*255]
        bigimg = cpu.tile_images(imgs, 2, 3)
        cv2.imshow("image filter results", bigimg)
        cv2.moveWindow("image filter results", 600, 0)

        plotsize = np.array([700, int(700.0/rgb.shape[1]*rgb.shape[0])])
        cv2.imshow("rgb", cv2.resize(rgb,(plotsize[0], plotsize[1])))
        cv2.moveWindow("rgb", 0, 0)
        cv2.waitKey(1000)

    good_xyz = xyz_w[good_mask]

    return clouds.downsample(good_xyz, .025)


def grabcut(rgb, depth, T_w_k):
    xyz_k = clouds.depth_to_xyz(depth, berkeley_pr2.f)
    xyz_w = xyz_k.dot(T_w_k[:3,:3].T) + T_w_k[:3,3][None,None,:]

    valid_mask = depth > 0

    import interactive_roi as ir
    xys = ir.get_polyline(rgb, "rgb")
    xy_corner1 = np.clip(np.array(xys).min(axis=0), [0,0], [639,479])
    xy_corner2 = np.clip(np.array(xys).max(axis=0), [0,0], [639,479])
    polymask = ir.mask_from_poly(xys)
    #cv2.imshow("mask",mask)
        
    xy_tl = np.array([xy_corner1, xy_corner2]).min(axis=0)
    xy_br = np.array([xy_corner1, xy_corner2]).max(axis=0)

    xl, yl = xy_tl
    w, h = xy_br - xy_tl
    mask = np.zeros((h,w),dtype='uint8')    
    mask[polymask[yl:yl+h, xl:xl+w] > 0] = cv2.GC_PR_FGD
    print mask.shape
    #mask[h//4:3*h//4, w//4:3*w//4] = cv2.GC_PR_FGD

    tmp1 = np.zeros((1, 13 * 5))
    tmp2 = np.zeros((1, 13 * 5))    
    cv2.grabCut(rgb[yl:yl+h, xl:xl+w, :],mask,(0,0,0,0),tmp1, tmp2,10,mode=cv2.GC_INIT_WITH_MASK)

    mask = mask % 2
    #mask = ndi.binary_erosion(mask, utils_images.disk(args.erode)).astype('uint8')
    contours = cv2.findContours(mask,cv2.RETR_LIST,cv2.CHAIN_APPROX_NONE)[0]
    cv2.drawContours(rgb[yl:yl+h, xl:xl+w, :],contours,-1,(0,255,0),thickness=2)
    
    cv2.imshow('rgb', rgb)
    print "press enter to continue"
    cv2.waitKey()

    zsel = xyz_w[yl:yl+h, xl:xl+w, 2]
    mask = (mask%2==1) & np.isfinite(zsel)# & (zsel - table_height > -1)
    mask &= valid_mask[yl:yl+h, xl:xl+w]
    
    xyz_sel = xyz_w[yl:yl+h, xl:xl+w,:][mask.astype('bool')]
    return clouds.downsample(xyz_sel, .01)
    #rgb_sel = rgb[yl:yl+h, xl:xl+w,:][mask.astype('bool')]
        


def extract_red_alphashape(cloud, robot):
    """
    extract red, get alpha shape, downsample
    """
    raise NotImplementedError
    
    # downsample cloud
    cloud_ds = cloudprocpy.downsampleCloud(cloud, .01)
    
    # transform into body frame
    xyz1_kinect = cloud_ds.to2dArray()
    xyz1_kinect[:,3] = 1
    T_w_k = berkeley_pr2.get_kinect_transform(robot)
    xyz1_robot = xyz1_kinect.dot(T_w_k.T)
    
    # compute 2D alpha shape
    xyz1_robot_flat = xyz1_robot.copy()
    xyz1_robot_flat[:,2] = 0 # set z coordinates to zero
    xyz1_robot_flatalphashape = cloudprocpy.computeAlphaShape(xyz1_robot_flat)
    
    # unfortunately pcl alpha shape func throws out the indices, so we have to use nearest neighbor search
    cloud_robot_flatalphashape = cloudprocpy.CloudXYZ()
    cloud_robot_flatalphashape.from2dArray(xyz1_robot_flatalphashape)
    cloud_robot_flat = cloudprocpy.CloudXYZ()
    cloud_robot_flat.from2dArray(xyz1_robot_flat)
    alpha_inds = cloudprocpy.getNearestNeighborIndices(xyz1_robot_flatalphashape, xyz1_robot_flat)

    xyz_robot_alphashape = xyz1_robot_flatalphashape[:,:3]
    
    # put back z coordinate
    xyz_robot_alphashape[:,2] = xyz1_robot[alpha_inds,2] 

    return xyz_robot_alphashape[:,:3]
