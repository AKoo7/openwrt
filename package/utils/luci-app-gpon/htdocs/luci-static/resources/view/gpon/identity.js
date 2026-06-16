// SPDX-License-Identifier: GPL-2.0-or-later
// LuCI view for per-board GPON ONU provisioning (/etc/config/gpon).
'use strict';
'require view';
'require form';
'require uci';
'require fs';
'require rpc';
'require poll';

/* Live ONU state + fiber RX/TX dBm, from /proc/gpon via the rpcd backend. */
var callGpon = rpc.declare({ object: 'gpon', method: 'status' });

/*
 * Parse "key=value\nkey=value\n..." output from rtk_factory optical_cal
 * into a plain object.
 */
function parseKV(text) {
	var r = {};
	(text || '').split('\n').forEach(function(l) {
		var eq = l.indexOf('=');
		if (eq > 0) r[l.slice(0, eq)] = l.slice(eq + 1);
	});
	return r;
}

/* Read the per-field factory values (what is in effect when a field is empty /
 * source=Factory) so we can show them as input placeholders + a copyable table. */
var FAC_KEYS = ['sn', 'mac', 'loid', 'loid_passwd', 'ploam_passwd', 'olt_mode', 'vendor_id'];

function factoryRead(key) {
	return fs.exec('/usr/sbin/rtk_factory', ['-p', 'config', key])
		.then(function(r) { return ((r && r.stdout) || '').trim(); })
		.catch(function() { return ''; });
}

function loadFactory() {
	return Promise.all(FAC_KEYS.map(factoryRead)).then(function(vals) {
		var o = {};
		FAC_KEYS.forEach(function(k, i) { o[k] = vals[i]; });
		return o;
	});
}

return view.extend({
	load: function () {
		return Promise.all([
			uci.load('gpon'),
			fs.exec('/usr/sbin/rtk_factory', ['-p', 'config', 'optical_cal'])
			  .catch(function() { return { stdout: '', stderr: '' }; }),
			L.resolveDefault(callGpon(), {}),
			loadFactory()
		]);
	},

	render: function (data) {
		var optOut = data[1] ? data[1].stdout : '';
		var cal = parseKV(optOut);
		var calOk = !!(cal.rssi_v0 || cal.mpd0);
		var live = data[2] || {};
		var fac = data[3] || {};

		function stateText(d) {
			return (d && d.onu_state_name && d.onu_state_name != 'unknown')
				? ('O' + d.onu_state + ' – ' + d.onu_state_name + (d.online ? ' (online)' : ''))
				: '—';
		}

		var m, s, o;

		m = new form.Map('gpon', _('GPON ONU Provisioning'),
			_('Per-board GPON identity. With source <em>Factory</em> the MAC, OUI and ' +
			  'serial number are read from this unit\'s own flash at boot, so one firmware ' +
			  'image serves the whole fleet. Switch to <em>Manual</em> to override MAC/SN. ' +
			  'LOID/PLOAM are operator settings and always editable. Save &amp; Apply ' +
			  're-provisions live without a reboot.'));

		s = m.section(form.NamedSection, 'identity', 'gpon', _('Identity'));
		s.addremove = false;

		o = s.option(form.ListValue, 'source', _('Source'));
		o.value('factory', _('Factory (read MAC/OUI/SN from flash)'));
		o.value('manual', _('Manual (override below)'));
		o.default = 'factory';

		o = s.option(form.Value, 'sn', _('GPON Serial Number'),
			_('e.g. XPON12345678'));
		o.depends('source', 'manual');
		o.placeholder = fac.sn || '';

		o = s.option(form.Value, 'mac', _('MAC Address'));
		o.datatype = 'macaddr';
		o.depends('source', 'manual');
		o.placeholder = fac.mac || '';

		o = s.option(form.Value, 'loid', _('LOID'));
		o.placeholder = fac.loid || '';

		o = s.option(form.Value, 'loid_passwd', _('LOID Password'));
		o.password = true;
		o.placeholder = fac.loid_passwd || '';

		o = s.option(form.Value, 'ploam_passwd', _('PLOAM Password'));
		o.password = true;
		o.placeholder = fac.ploam_passwd || '';

		o = s.option(form.ListValue, 'omci_olt_mode', _('OMCI OLT Mode'));
		o.value('', _('Use factory value') + (fac.olt_mode ? ' (' + fac.olt_mode + ')' : ''));
		o.value('0', _('0 - Default'));
		o.value('1', _('1 - Huawei'));
		o.value('2', _('2 - ZTE'));
		o.value('3', _('3 - Customized'));

		o = s.option(form.Value, 'pon_vendor_id', _('PON Vendor ID'),
			_('4 chars, e.g. XPON; empty = factory value'));
		o.placeholder = fac.vendor_id || '';

		/* Optical transceiver calibration (read-only, from rtl8290b.data) */
		var optSec = m.section(form.TypedSection, 'gpon', _('Optical Transceiver (rtl8290b)'));
		optSec.anonymous = true;
		optSec.addremove = false;

		var optDiv = E('div', { 'class': 'cbi-section' }, [
			E('h3', {}, [ _('Optical Transceiver (rtl8290b)') ]),
			E('div', { 'class': 'cbi-section-descr' }, [
				_('Factory calibration data from the config flash partition. ' +
				  'ONU state and live RX/TX power below refresh automatically every 5s ' +
				  '(the BOSA publishes optical power even before O5).')
			]),
			E('table', { 'class': 'table cbi-section-table' }, [
				E('tr', { 'class': 'tr table-titles' }, [
					E('th', { 'class': 'th' }, [ _('Parameter') ]),
					E('th', { 'class': 'th' }, [ _('Value') ])
				]),
				E('tr', { 'class': 'tr' }, [
					E('td', { 'class': 'td' }, [ _('Calibration data') ]),
					E('td', { 'class': 'td' }, [
						calOk ? _('Present') : _('Not found in flash')
					])
				]),
				calOk ? E('tr', { 'class': 'tr' }, [
					E('td', { 'class': 'td' }, [ _('RSSI ref (V0)') ]),
					E('td', { 'class': 'td' }, [ cal.rssi_v0 || '?' ])
				]) : E([]),
				calOk ? E('tr', { 'class': 'tr' }, [
					E('td', { 'class': 'td' }, [ _('MPD0 ref (TX)') ]),
					E('td', { 'class': 'td' }, [ cal.mpd0 || '?' ])
				]) : E([]),
				calOk ? E('tr', { 'class': 'tr' }, [
					E('td', { 'class': 'td' }, [ _('RX LOS assert / de-assert (raw)') ]),
					E('td', { 'class': 'td' }, [
						(cal.rx_los_th || '?') + ' / ' + (cal.rx_delos_th || '?')
					])
				]) : E([]),
				calOk ? E('tr', { 'class': 'tr' }, [
					E('td', { 'class': 'td' }, [ _('Temp offset (°C)') ]),
					E('td', { 'class': 'td' }, [ cal.temp_off || '0' ])
				]) : E([]),
				E('tr', { 'class': 'tr' }, [
					E('td', { 'class': 'td' }, [ _('ONU state') ]),
					E('td', { 'class': 'td', 'id': 'gpon-live-state' }, [ stateText(live) ])
				]),
				E('tr', { 'class': 'tr' }, [
					E('td', { 'class': 'td' }, [ _('Live RX power') ]),
					E('td', { 'class': 'td', 'id': 'gpon-live-rx' }, [
						(live.optic_rx_dbm || 'n/a') + ' dBm'
					])
				]),
				E('tr', { 'class': 'tr' }, [
					E('td', { 'class': 'td' }, [ _('Live TX power') ]),
					E('td', { 'class': 'td', 'id': 'gpon-live-tx' }, [
						(live.optic_tx_dbm || 'n/a') + ' dBm'
					])
				])
			])
		]);

		var facRows = [
			[ _('GPON Serial Number'), fac.sn ],
			[ _('MAC Address'), fac.mac ],
			[ _('LOID'), fac.loid ],
			[ _('LOID Password'), fac.loid_passwd ],
			[ _('PLOAM Password'), fac.ploam_passwd ],
			[ _('OMCI OLT Mode'), fac.olt_mode ],
			[ _('PON Vendor ID'), fac.vendor_id ]
		];
		var facDiv = E('div', { 'class': 'cbi-section' }, [
			E('h3', {}, [ _('Factory identity values') ]),
			E('div', { 'class': 'cbi-section-descr' }, [
				_('Per-board values read from this unit\'s flash — in effect when the matching ' +
				  'field above is left empty. Read-only, so you can copy them and notice if a ' +
				  'value that should be constant ever changes (e.g. memory corruption).')
			]),
			E('table', { 'class': 'table cbi-section-table' },
				[ E('tr', { 'class': 'tr table-titles' }, [
					E('th', { 'class': 'th' }, [ _('Field') ]),
					E('th', { 'class': 'th' }, [ _('Factory value') ])
				]) ].concat(facRows.map(function(r) {
					return E('tr', { 'class': 'tr' }, [
						E('td', { 'class': 'td' }, [ r[0] ]),
						E('td', { 'class': 'td', 'style': 'font-family:monospace;user-select:all' },
							[ (r[1] && r[1].length) ? r[1] : '—' ])
					]);
				}))
			)
		]);

		return Promise.resolve(m.render()).then(function(node) {
			node.appendChild(facDiv);
			node.appendChild(optDiv);
			/* Live refresh: a value correct at O5 that a bug later corrupts will
			 * visibly change here, since this keeps showing the current value. */
			poll.add(function() {
				return L.resolveDefault(callGpon(), {}).then(function(d) {
					d = d || {};
					var rx = document.getElementById('gpon-live-rx');
					var tx = document.getElementById('gpon-live-tx');
					var st = document.getElementById('gpon-live-state');
					if (rx) rx.textContent = (d.optic_rx_dbm || 'n/a') + ' dBm';
					if (tx) tx.textContent = (d.optic_tx_dbm || 'n/a') + ' dBm';
					if (st) st.textContent = stateText(d);
				});
			}, 5);
			return node;
		});
	}
});
