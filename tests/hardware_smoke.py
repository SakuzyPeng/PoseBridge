"""Explicit hardware validation via the actual C ABI; never calibrates, zeroes, saves or resets.

--exercise-config temporarily changes rate/output/algorithm and restores their observed values.
Without that flag, the checker only inspects and reads the currently configured stream.
--render-library optionally verifies the same poses through MacinRender's real C ABI.
"""
import argparse
import ctypes as c
import importlib.util
import json
from pathlib import Path
import socket
import struct
import sys
import time

ROOT = Path(__file__).resolve().parents[1]

class Bridge:
    def __init__(self, library):
        self.lib = c.CDLL(str(Path(library).resolve())); self.ctx = c.c_void_p()
        for name, args in {
            'pb_context_create': [c.POINTER(c.c_void_p)], 'pb_context_destroy': [c.c_void_p],
            'pb_configure': [c.c_void_p,c.c_char_p,c.c_uint32], 'pb_start':[c.c_void_p], 'pb_stop':[c.c_void_p],
            'pb_inspect_start':[c.c_void_p], 'pb_device_command':[c.c_void_p,c.c_char_p,c.c_uint32],
            'pb_snapshot_json':[c.c_void_p,c.c_void_p,c.c_uint32,c.POINTER(c.c_uint32)],
        }.items():
            fn=getattr(self.lib,name); fn.argtypes=args; fn.restype=c.c_int
        assert self.lib.pb_abi_version()==400
        assert self.lib.pb_context_create(c.byref(self.ctx))==0
    def close(self):
        if self.ctx: assert self.lib.pb_context_destroy(self.ctx)==0; self.ctx=c.c_void_p()
    def snapshot(self):
        buffer=c.create_string_buffer(32768);needed=c.c_uint32()
        assert self.lib.pb_snapshot_json(self.ctx,buffer,len(buffer),c.byref(needed))==0
        value=json.loads(buffer.value)
        assert value['schema']==4
        return value
    def configure(self, config):
        data=json.dumps(config).encode();assert self.lib.pb_configure(self.ctx,data,len(data))==0
    def operation(self, command=None):
        if command is None: rc=self.lib.pb_inspect_start(self.ctx)
        else:
            data=json.dumps(command).encode();rc=self.lib.pb_device_command(self.ctx,data,len(data))
        assert rc==0,(rc,self.snapshot())
        deadline=time.monotonic()+20
        while True:
            value=self.snapshot()
            if value['status']['state']=='complete':
                assert value['operation']['outcome']=='succeeded',value
                return value
            assert value['status']['state'] not in ('failed','stopped'),value
            assert time.monotonic()<deadline,value
            time.sleep(.02)
    def stop(self): assert self.lib.pb_stop(self.ctx)==0

def decode(data):
    assert len(data)<=8192
    def string(offset):
        end=data.index(0,offset);following=(end+4)&~3
        assert not any(data[end:following]);return data[offset:end].decode('utf-8'),following
    address,pos=string(0);tags,pos=string(pos)
    if address in ('/posebridge/info','/posebridge/status'):
        assert tags==',s';value,end=string(pos);assert end==len(data)
        meta=json.loads(value);assert meta['schema']==3
        return address,meta
    assert address=='/posebridge/quaternion' and tags==',ishhhhhhhhihhffff'
    version=struct.unpack_from('>i',data,pos)[0];pos+=4;source,pos=string(pos)
    values=struct.unpack_from('>qqqqqqqqiqqffff',data,pos)
    assert version==3 and pos+struct.calcsize('>qqqqqqqqiqqffff')==len(data)
    names=['instance_id','session_id','sequence','tx_sequence','reference_epoch','metadata_revision','received_ns','age_at_send_ns','kind','sample_ms','clock_epoch']
    result=dict(zip(names,values[:11]));result['source_id']=source;result['quaternion']=values[11:]
    assert result['age_at_send_ns']<500000000 and abs(sum(v*v for v in values[11:])-1)<1e-5
    return address,result

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--library',type=Path,default=ROOT/'target/release'/('posebridge_capi.dll' if sys.platform=='win32' else 'libposebridge_capi.dylib'))
    parser.add_argument('--transport',choices=['usb','ble'],required=True)
    parser.add_argument('--device');parser.add_argument('--port');parser.add_argument('--seconds',type=float,default=4)
    parser.add_argument('--exercise-config',action='store_true');parser.add_argument('--output',type=Path)
    parser.add_argument('--render-library',type=Path);parser.add_argument('--render-helper',type=Path)
    parser.add_argument('--expect-reconnect',action='store_true',help='Read-only capture; operator must physically interrupt and restore the selected link during the window.')
    args=parser.parse_args()
    if args.expect_reconnect and args.exercise_config: parser.error('hotplug capture must not change configuration')
    source={'kind':'usb','port':args.port,'baud':115200} if args.transport=='usb' else {'kind':'ble','device_id':args.device}
    if not (args.port if args.transport=='usb' else args.device):parser.error('explicit port/device required')
    config={'source_id':'hardware-test','source':source,'mounting':{'right':-2,'forward':1,'up':3}}
    bridge=Bridge(args.library);report={'transport':args.transport,'controls_exercised':args.exercise_config};before=None;changed=False;receiver=None
    try:
        bridge.configure(config);initial=bridge.operation();before=initial['descriptor']['device'];report['before']=before
        assert before['valid'];print('INSPECT',json.dumps(before),flush=True)
        if args.exercise_config:
            assert before['rate_hz'] in [1,2,5,10,20,50,100,200] and before['output_register'] in [0x61,0x81,0x84,0xa4]
            assert before['algorithm_register'] in [0,1]
            changed=True
            alternate='six_axis' if before['algorithm_register']==0 else 'nine_axis'
            bridge.operation({'action':'algorithm','mode':alternate})
            bridge.operation({'action':'algorithm','mode':'nine_axis' if before['algorithm_register']==0 else 'six_axis'})
            bridge.operation({'action':'rate','hz':100})
            bridge.operation({'action':'output','format':'timestamp_gyro_quaternion'})
            observed=bridge.operation()['descriptor']['device'];assert observed['rate_hz']==100 and observed['output_register']==0xa4
            report['configured']=observed
        else: observed=before
        config['pose_input']='stream_quaternion' if observed['output_register'] in (4,0x84,0xa4) else 'euler'
        udp=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);udp.bind(('127.0.0.1',0));udp.setblocking(False)
        config['osc']={'target':f'127.0.0.1:{udp.getsockname()[1]}','max_rate_hz':200,'format':'quaternion'}
        if args.render_library:
            assert args.render_helper
            spec=importlib.util.spec_from_file_location('native_receiver',args.render_helper);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
            receiver=module.Receiver(args.render_library,'hardware-test')
            # Same captured UDP message is forwarded unchanged to the actual C ABI receiver.
            render_target=('127.0.0.1',receiver.status().bound_port)
        bridge.configure(config);assert bridge.lib.pb_start(bridge.ctx)==0
        deadline=time.monotonic()+25+args.seconds;finish=None;poses=[];metas=[];states=set();sessions=set();seen={};last_tx=0;render_matches=0;recovered_at=None
        try:
            while True:
                snapshot=bridge.snapshot();states.add(snapshot['status']['state'])
                value=snapshot['pose']
                if value and value['fresh']:
                    key=(int(value['session_id']),int(value['sequence']));seen[key]=value;sessions.add(key[0])
                    if finish is None:finish=time.monotonic()+args.seconds;print('CAPTURE_READY',flush=True)
                assert snapshot['status']['state']!='failed',snapshot
                while True:
                    try:data=udp.recv(8193)
                    except BlockingIOError:break
                    address,decoded=decode(data)
                    if receiver: udp.sendto(data,render_target)
                    if address.endswith('quaternion'):
                        assert decoded['tx_sequence']==last_tx+1;last_tx=decoded['tx_sequence'];poses.append(decoded)
                    else: metas.append(decoded)
                if receiver:
                    target=receiver.pose();key=(target.source_session_id,target.source_sequence)
                    if target.has_pose and key in seen:
                        src=seen[key];assert target.source_received_ns==int(src['received_ns'])
                        assert target.reference_epoch==int(src['reference_epoch']) and target.metadata_revision==int(src['metadata_revision'])
                        assert max(abs(a-b) for a,b in zip(target.quaternion,src['quaternion_xyzw']))<1e-6
                        render_matches+=1
                now=time.monotonic()
                if args.expect_reconnect and len(sessions)>=2 and snapshot['status']['state']=='active':
                    if recovered_at is None: recovered_at=now
                    if now-recovered_at>=2: break
                if finish and now>=finish:break
                assert now<deadline,snapshot
                time.sleep(.002)
            bridge.stop()
            terminal=time.monotonic()+.15
            while time.monotonic()<terminal:
                try:data=udp.recv(8193)
                except BlockingIOError:time.sleep(.005);continue
                address,decoded=decode(data)
                if receiver:udp.sendto(data,render_target)
                if address.endswith('quaternion'):poses.append(decoded)
                else:metas.append(decoded)
            final=bridge.snapshot()
            assert len(poses)>5 and any(m['kind']=='info' for m in metas)
            assert any(m['kind']=='status' and m['status']['state']=='stopped' for m in metas)
            matched=0
            for p in poses:
                key=(p['session_id'],p['sequence'])
                if key in seen:
                    src=seen[key];assert p['received_ns']==int(src['received_ns']);matched+=1
                    stamp=src['sample_time'];assert p['sample_ms']==(int(stamp['time_ms']) if stamp else 0)
            assert matched>3
            if args.expect_reconnect:assert len(sessions)>=2 and int(final['status']['reconnect_count'])>=1,(sessions,final)
            report.update(pose_packets=len(poses),telemetry_packets=len(metas),matched_snapshots=matched,render_matches=render_matches,
                states=sorted(states),sessions=len(sessions),final_status=final['status'],final_descriptor=final['descriptor'])
            if receiver:
                time.sleep(.55);s=receiver.snapshot();assert not s['fresh'] and int(s['rejected_packets'])==0
                assert s['info_matches_pose'];report['render']=s
        finally:udp.close();bridge.stop()
    finally:
        try:
            if changed:
                bridge.stop();bridge.configure({'source':source,'mounting':{'right':-2,'forward':1,'up':3}})
                profiles={0x61:'motion',0x81:'timestamp_euler',0x84:'timestamp_quaternion',0xa4:'timestamp_gyro_quaternion'}
                bridge.operation({'action':'rate','hz':int(before['rate_hz'])})
                bridge.operation({'action':'output','format':profiles[before['output_register']]})
                bridge.operation({'action':'algorithm','mode':'nine_axis' if before['algorithm_register']==0 else 'six_axis'})
                restored=bridge.operation()['descriptor']['device'];report['restored']=restored
                for key in ['rate_register','output_register','algorithm_register']:assert restored[key]==before[key]
                print('RESTORED',json.dumps(restored),flush=True)
        finally:
            bridge.close()
            if receiver:receiver.close()
            if args.output:args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({'result':'PASS','transport':args.transport,'pose_packets':report['pose_packets'],'matched':report['matched_snapshots'],'sessions':report['sessions']}),flush=True)
if __name__=='__main__':main()
