// SPDX-License-Identifier: GPL-2.0-or-later
// LuCI view for per-board GPON ONU provisioning (/etc/config/gpon).
'use strict';
'require view';
'require form';
'require uci';
'require fs';

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

return view.extend({
	load: function () {
		return Promise.all([
			uci.load('gpon'),
			fs.exec('/usr/sbin/rtk_factory', ['-p', 'config', 'optical_cal'])
			  .catch(function() { return { stdout: '', stderr: '' }; })
		]);
	},

	render: function (data) {
		var optOut = data[1] ? data[1].stdout : '';
		var cal = parseKV(optOut);
		var calOk = !!(cal.rssi_v0 || cal.mpd0);

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

		o = s.option(form.Value, 'mac', _('MAC Address'));
		o.datatype = 'macaddr';
		o.depends('source', 'manual');

		o = s.option(form.Value, 'loid', _('LOID'));

		o = s.option(form.Value, 'loid_passwd', _('LOID Password'));
		o.password = true;

		o = s.option(form.Value, 'ploam_passwd', _('PLOAM Password'));
		o.password = true;

		o = s.option(form.ListValue, 'omci_olt_mode', _('OMCI OLT Mode'));
		o.value('', _('Use factory value'));
		o.value('0', _('0 - Default'));
		o.value('1', _('1 - Huawei'));
		o.value('2', _('2 - ZTE'));
		o.value('3', _('3 - Customized'));

		o = s.option(form.Value, 'pon_vendor_id', _('PON Vendor ID'),
			_('4 chars, e.g. XPON; empty = factory value'));

		/* Optical transceiver calibration (read-only, from rtl8290b.data) */
		var optSec = m.section(form.TypedSection, 'gpon', _('Optical Transceiver (rtl8290b)'));
		optSec.anonymous = true;
		optSec.addremove = false;

		var optDiv = E('div', { 'class': 'cbi-section' }, [
			E('h3', {}, [ _('Optical Transceiver (rtl8290b)') ]),
			E('div', { 'class': 'cbi-section-descr' }, [
				_('Factory calibration data from the config flash partition. ' +
				  'Live RX/TX power requires the PON stack to be running.')
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
					E('td', { 'class': 'td' }, [ _('Live RX power') ]),
					E('td', { 'class': 'td' }, [ _('N/A — requires PON driver') ])
				]),
				E('tr', { 'class': 'tr' }, [
					E('td', { 'class': 'td' }, [ _('Live TX power') ]),
					E('td', { 'class': 'td' }, [ _('N/A — requires PON driver') ])
				])
			])
		]);

		return Promise.resolve(m.render()).then(function(node) {
			node.appendChild(optDiv);
			return node;
		});
	}
});
