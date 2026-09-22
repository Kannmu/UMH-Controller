import sys, time
sys.path.insert(0, r'D:\Data\OneDrive\Projects\UMH\Software\UMH Controller\Utiles\bench')
from umh_link import UmhLink
link = UmhLink('COM4')
p = link.get_profile()
print('device', p.model, p.firmware, 'focused_am', p.has_focused_am)
link.audio_configure([(0, 0, 1_000_000, 0, 255)], 20000, 512, 255)
link.audio_start()
tones = bytes([64 + int(60 * __import__('math').sin(i * 0.05)) for i in range(256)])
seq = 0
sent = 0
t0 = time.monotonic()
for k in range(100000):
    t = time.monotonic() - t0
    target = int(t * 20000)
    while sent < target:
        link.audio_data(tones, seq)
        seq += 1
        sent += 256
    if t >= 5.0:
        break
    time.sleep(0.002)
st = link.get_audio_status()
print('under', st.underrun_count, 'over', st.overrun_count, 'loss', st.packet_loss_count, 'samples', st.rendered_samples, 'ppm', st.clock_correction_ppm, 'max_svc_us', st.max_service_us)
link.audio_stop()
link.close()
