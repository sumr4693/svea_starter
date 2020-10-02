
"""
Module to perform mean max data pooling with 2D or 3D data
"""

import numpy as np

__team__ = "Spice Level 5"
__maintainers__ = "Mikel Zhobro, Mikael Glamheden, Karl Hemlin, Subramanian Murali Ram, Mustafa Al-Janabi"
__status__ = "Development"

def asStride(arr,sub_shape,stride):
    """
    Get a strided sub-matrices view of an ndarray.
    See also skimage.util.shape.view_as_windows()
    """
    s0,s1=arr.strides[:2]
    m1,n1=arr.shape[:2]
    m2,n2=sub_shape
    view_shape=(1+(m1-m2)//stride[0],1+(n1-n2)//stride[1],m2,n2)+arr.shape[2:]
    strides=(stride[0]*s0,stride[1]*s1,s0,s1)+arr.strides[2:]
    subs=np.lib.stride_tricks.as_strided(arr,view_shape,strides=strides)
    return subs

def poolingOverlap(mat,ksize,stride=None,method='max',pad=True):
    """
    Overlapping pooling on 2D or 3D data.

    :param mat: Input array to pool.
    :type mat: ndarray
    :param ksize: Kernel size in (ky, kx).
    :type ksize: tuple of 2
    :param stride: Stride of pooling window. If None, same as ksize (non-overlapping pooling).
    :type stride: tuple of 2 or None
    :param method: 'max' for max-pooling,
                   'mean' for mean-pooling.
    :type method: str
    :param pad: pad mat or not. If no pad, output has size
                (n-f)//s+1, n being mat size, f being kernel size, s stride.
                if pad, output has size ceil(n/s).
    :type pad: bool
    :return: result     Pooled matrix
    :rtype: ndarray
    """

    m, n = mat.shape[:2]
    ky,kx=ksize
    if stride is None:
        stride=(ky,kx)
    sy,sx=stride

    _ceil=lambda x,y: int(np.ceil(x/float(y)))

    if pad:
        ny=_ceil(m,sy)
        nx=_ceil(n,sx)
        size=((ny-1)*sy+ky, (nx-1)*sx+kx) + mat.shape[2:]
        mat_pad=np.full(size,np.nan)
        mat_pad[:m,:n,...]=mat
    else:
        mat_pad=mat[:(m-ky)//sy*sy+ky, :(n-kx)//sx*sx+kx, ...]

    view=asStride(mat_pad,ksize,stride)

    if method=='max':
        result=np.nanmax(view,axis=(2,3))
    else:
        result=np.nanmean(view,axis=(2,3))
    return result