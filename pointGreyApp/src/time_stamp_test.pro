; This program reads the frame counter, camera timestamp and EPICS timestamp from a netCDF file streamed by areaDetector
; It looks for missing frame numbers, the jitter on the camera and EPICS timestamps, and any systematic differences
; between the camera and EPICS timestamps
;
data = 0
; 001 is 100 Hz
; 002 is 158 Hz
; 003 is 160 Hz
data = read_nd_netcdf('camera_time_004.nc', attributes=attributes)

frame_number  = *attributes[0].pValue
camera_time   = *attributes[1].pValue
epics_seconds = *attributes[2].pValue
epics_nsec    = *attributes[3].pValue
epics_time = double(epics_seconds) + epics_nsec/1.e9

frame_number_delta = (frame_number - shift(frame_number, 1))[1:*]
iplot, frame_number_delta, ytitle='Frame number delta', yrange=[0,2]

camera_time_delta = (camera_time - shift(camera_time, 1))[1:*]
iplot, camera_time_delta, ytitle='Camera time delta'

epics_time_delta = (epics_time - shift(epics_time, 1))[1:*]
iplot, epics_time_delta, ytitle='EPICS time delta'

iplot, (camera_time - camera_time[0]) - (epics_time - epics_time[0]), ytitle='Camera time - EPICS time'

end



